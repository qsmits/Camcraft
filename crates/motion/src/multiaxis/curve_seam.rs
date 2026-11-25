//! The user's start and exit points on the drive-curve contour (#3330,
//! the Multi-Axis Contour slice of #1448). A pick acts on the drive
//! curve nearest it, one curve for each pick:
//!
//! - **Closed curve, start:** the curve is re-sequenced to begin at its
//!   point nearest the pick (a new vertex on the segment when the foot
//!   falls between two vertices).
//! - **Closed curve, exit:** the curve cuts its whole lap from its seam,
//!   then runs on along its own points to its point nearest the exit
//!   pick, and the tool leaves there.
//! - **Open curve:** a start pick selects the end the cut starts from
//!   (the nearer end), and an exit pick the end it leaves at. An exit
//!   pick that is not near the far end of the cut is refused.
//!
//! The cut geometry does not change: the same curve points are walked,
//! only the order, the direction of an open curve, and a re-trace over
//! cut material on an exit lap. What a pick cannot keep is refused by
//! name, never moved: a pick farther than 1.5 tool diameters from every
//! drive curve, an open curve's end farther than that from its pick,
//! a start and an exit at the same end, and a plunge or retract column
//! that crosses the part ([`column_clear`]) at either end of a curve
//! that a pick changed - a pick that reverses an open curve moves its
//! other end too ([`Seated::columns`]).
//!
//! Provenance: first principles - the closest point on a segment
//! (orthogonal projection clamped to the segment), the rotation of a
//! closed polyline's vertex list, and the exact segment-to-triangle
//! distance (Ericson, "Real-Time Collision Detection" 5.1) for the
//! sphere swept along a vertical line; the seam position is the common
//! contour-entry vocabulary of published CAM practice. No vendor
//! material consulted.

use crate::geometry::{Point3, Triangle};
use crate::machining2d::GenError;
use crate::path_picks::{PathPick, PathPicks, PickRole};
use crate::stock_sim::dist_seg_tri;

use super::avoid::{shank_hits_at, Shank};

/// The start and exit points the drive-curve contour reads, in the
/// frame of its chains. Both `None` is the automatic path of every
/// earlier version, byte for byte.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CurveSeam {
    /// The nearest drive curve starts at its point nearest this pick.
    pub start: Option<PathPick>,
    /// The nearest drive curve leaves at its point nearest this pick.
    pub exit: Option<PathPick>,
}

impl CurveSeam {
    /// The seam of an operation's picks: the first start and the first
    /// exit. A pick with a Z is read in 3D, a pick without one in XY.
    #[must_use]
    pub fn of(picks: &PathPicks) -> Self {
        Self { start: picks.first(PickRole::Start), exit: picks.first(PickRole::Exit) }
    }

    /// No pick of either role.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.start.is_none() && self.exit.is_none()
    }
}

/// The picks that one chain took, for the column law of the walk.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Seated {
    pub start: Option<PathPick>,
    pub exit: Option<PathPick>,
}

impl Seated {
    /// The columns the walk judges on this chain, as `(column, role,
    /// pick)`: the plunge (`Start`) and the retract (`Exit`) column at
    /// BOTH ends of a chain that took either pick, each named by the
    /// pick of its own role, else by the other pick. A start pick that
    /// reverses an open curve also moves its exit, and an exit pick
    /// that reverses it also moves its start (#4050 review); a chain
    /// with no pick yields nothing, so the automatic path is unchanged.
    pub(super) fn columns(&self) -> impl Iterator<Item = (PickRole, PickRole, PathPick)> {
        let by = |own: Option<PathPick>, own_role: PickRole, other: Option<PathPick>, other_role: PickRole| {
            own.map(|p| (own_role, p)).or(other.map(|p| (other_role, p)))
        };
        [
            (PickRole::Start, by(self.start, PickRole::Start, self.exit, PickRole::Exit)),
            (PickRole::Exit, by(self.exit, PickRole::Exit, self.start, PickRole::Start)),
        ]
        .into_iter()
        .filter_map(|(col, named)| named.map(|(role, p)| (col, role, p)))
    }
}

/// The farthest a pick may sit from its drive curve (and from the end
/// of an open curve that it selects): one and a half tool diameters,
/// the 2D contour's slack for a click beside the edge.
fn reach(tool_diameter: f64) -> f64 {
    tool_diameter * 1.5
}

/// "start point (x, y)" or "exit point (x, y, z)", the refusals' lead.
pub(super) fn named(role: PickRole, p: &PathPick) -> String {
    match p.z {
        Some(z) => format!("{} point ({:.3}, {:.3}, {:.3})", role.word(), p.xy.x, p.xy.y, z),
        None => format!("{} point ({:.3}, {:.3})", role.word(), p.xy.x, p.xy.y),
    }
}

/// The distance of `q` from the pick: in 3D when the pick states a Z,
/// in XY when it does not.
fn gap(p: &PathPick, q: Point3) -> f64 {
    let (dx, dy) = (q.x - p.xy.x, q.y - p.xy.y);
    let dz = p.z.map_or(0.0, |z| q.z - z);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// The closest point to the pick on segment `a -> b`: `(t, point)`,
/// projected in 3D or in XY as [`gap`] reads.
fn foot(p: &PathPick, a: Point3, b: Point3) -> (f64, Point3) {
    let d = Point3::new(b.x - a.x, b.y - a.y, if p.z.is_some() { b.z - a.z } else { 0.0 });
    let w = Point3::new(p.xy.x - a.x, p.xy.y - a.y, p.z.map_or(0.0, |z| z - a.z));
    let dd = d.dot(d);
    let t = if dd > 0.0 && dd.is_finite() { (w.dot(d) / dd).clamp(0.0, 1.0) } else { 0.0 };
    (t, Point3::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t))
}

/// The closest point to the pick on the polyline `pts` (a ring carries
/// its closing vertex): `(segment, t, point, distance)`, the first
/// segment winning a tie. `None` when no answer is finite.
fn nearest(p: &PathPick, pts: &[Point3]) -> Option<(usize, f64, Point3, f64)> {
    let mut best: Option<(usize, f64, Point3, f64)> = None;
    for (i, w) in pts.windows(2).enumerate() {
        let (t, f) = foot(p, w[0], w[1]);
        let d = gap(p, f);
        if d.is_finite() && best.is_none_or(|b| d < b.3) {
            best = Some((i, t, f, d));
        }
    }
    best
}

/// The drive curve nearest the pick, refused by name when even that
/// one is farther than the reach.
fn chain_of(role: PickRole, p: &PathPick, live: &[(Vec<Point3>, bool)], d: f64) -> Result<usize, GenError> {
    let mut best: Option<(usize, f64)> = None;
    for (ci, (pts, _)) in live.iter().enumerate() {
        if let Some((_, _, _, g)) = nearest(p, pts)
            && best.is_none_or(|b| g < b.1)
        {
            best = Some((ci, g));
        }
    }
    match best {
        Some((ci, g)) if g <= reach(d) => Ok(ci),
        Some((_, g)) => Err(GenError::Invalid(format!(
            "{}: the nearest drive curve stands {g:.3} mm away, farther than {:.3} mm (1.5 tool \
             diameters) - pick on or beside a drive curve, or clear the {} point",
            named(role, p),
            reach(d),
            role.word()
        ))),
        None => Err(GenError::Invalid(format!("{}: no drive curve to read it on", named(role, p)))),
    }
}

/// The ring `pts` (closing vertex included) as the vertex list from
/// segment `i` at `t`: a foot within `snap` of a vertex starts there,
/// any other foot becomes a new first vertex. The ring stays closed.
fn rotate(pts: &[Point3], i: usize, t: f64, f: Point3, snap: f64) -> Vec<Point3> {
    let n = pts.len().saturating_sub(1);
    let near = |q: Point3| (f - q).dot(f - q).sqrt() <= snap;
    let (first, lead) = if near(pts[i]) || t <= 0.0 {
        (i, None)
    } else if near(pts[i + 1]) || t >= 1.0 {
        ((i + 1) % n, None)
    } else {
        ((i + 1) % n, Some(f))
    };
    let mut ring: Vec<Point3> = lead.into_iter().collect();
    ring.extend((0..n).map(|k| pts[(first + k) % n]));
    let close = ring[0];
    ring.push(close);
    ring
}

/// Re-sequences the drive curves for the seam's picks. `live` holds
/// each walkable chain as `contour_curve` built it: oriented for the
/// climb law, a ring closed on its first point. Returns the picks that
/// each chain took. An empty seam leaves `live` as it is.
pub(super) fn apply(seam: &CurveSeam, live: &mut [(Vec<Point3>, bool)], tool_d: f64, snap: f64) -> Result<Vec<Seated>, GenError> {
    let mut out = vec![Seated::default(); live.len()];
    if seam.is_empty() {
        return Ok(out);
    }
    let r = reach(tool_d);
    if let Some(p) = seam.start {
        let ci = chain_of(PickRole::Start, &p, live, tool_d)?;
        let (pts, ring) = &mut live[ci];
        if *ring {
            if let Some((i, t, f, _)) = nearest(&p, pts) {
                *pts = rotate(pts, i, t, f, snap);
            }
        } else {
            let (d0, d1) = (gap(&p, pts[0]), gap(&p, pts[pts.len() - 1]));
            let (end, d) = if d1 < d0 { (pts[pts.len() - 1], d1) } else { (pts[0], d0) };
            if d > r {
                return Err(far_end(PickRole::Start, &p, ci, end, d, r));
            }
            if d1 < d0 {
                pts.reverse();
            }
        }
        out[ci].start = Some(p);
    }
    if let Some(p) = seam.exit {
        let ci = chain_of(PickRole::Exit, &p, live, tool_d)?;
        let started = out[ci].start.is_some();
        let (pts, ring) = &mut live[ci];
        if *ring {
            if let Some((i, t, f, _)) = nearest(&p, pts) {
                run_on(pts, ring, i, t, f, snap);
            }
        } else {
            let last = pts.len() - 1;
            let (d0, d1) = (gap(&p, pts[0]), gap(&p, pts[last]));
            if started && d0 < d1 {
                return Err(GenError::Invalid(format!(
                    "{}: it is nearer the end of open drive curve {} where the start point begins the \
                     cut - an open curve leaves at its other end; pick near that end, or clear the \
                     exit point",
                    named(PickRole::Exit, &p),
                    ci + 1
                )));
            }
            if !started && d0 < d1 {
                pts.reverse();
            }
            let d = d0.min(d1);
            if d > r {
                return Err(far_end(PickRole::Exit, &p, ci, pts[last], d, r));
            }
        }
        out[ci].exit = Some(p);
    }
    Ok(out)
}

/// An open curve's end farther than the reach from the pick that
/// selects it.
fn far_end(role: PickRole, p: &PathPick, ci: usize, end: Point3, d: f64, r: f64) -> GenError {
    GenError::Invalid(format!(
        "{}: drive curve {} is open and its cut can {} only at an end; its nearest end \
         ({:.3}, {:.3}, {:.3}) stands {d:.3} mm away, farther than {r:.3} mm - pick near an end, or \
         clear the {} point",
        named(role, p),
        ci + 1,
        if role == PickRole::Start { "start" } else { "leave" },
        end.x,
        end.y,
        end.z,
        role.word()
    ))
}

/// Carries the ring `pts` (seam to seam) on along its own points to the
/// foot `f` on segment `i`: the whole lap, then the re-trace. A foot at
/// the seam keeps the closed ring as it is. A carried ring is walked as
/// an open run, which ends at the foot.
fn run_on(pts: &mut Vec<Point3>, ring: &mut bool, i: usize, t: f64, f: Point3, snap: f64) {
    let near = |q: Point3| (f - q).dot(f - q).sqrt() <= snap;
    let n = pts.len() - 1;
    let (upto, tail) = if near(pts[i]) || t <= 0.0 {
        (i, None)
    } else if near(pts[i + 1]) || t >= 1.0 {
        (i + 1, None)
    } else {
        (i, Some(f))
    };
    if upto % n == 0 && tail.is_none() {
        return;
    }
    let lap: Vec<Point3> = pts[1..=upto].to_vec();
    pts.extend(lap);
    pts.extend(tail);
    *ring = false;
}

/// Is the vertical column over ball centre `cl` (posed `pose`, the tip
/// rising to `hop_z`) clear of the part? The ball swept up the whole
/// line must keep `r - band` from every facet: the exact distance from
/// the segment to each facet ([`dist_seg_tri`]), so no ledge can stand
/// between two samples, however small the ball (#4050 review). With a
/// `shank`, the shank must touch nothing either: its test is a column
/// that reaches up from the band floor, and that floor only rises with
/// the ball, so the lowest pose answers for the whole rise. A keep of
/// zero or less cannot be judged and is refused. The plunge descends
/// this column, and the retract rises it.
#[allow(clippy::too_many_arguments)] // the flat geometry a column question needs
pub(super) fn column_clear(
    cl: Point3,
    pose: [f64; 3],
    hop_z: f64,
    r: f64,
    band: f64,
    facets: &[Triangle],
    boxes: &[[f64; 6]],
    shank: Option<&Shank>,
) -> bool {
    let top = hop_z + r * pose[2];
    let rise = top - cl.z;
    if !(rise.is_finite() && rise > 0.0) {
        return true;
    }
    let keep = r - band;
    if !(keep.is_finite() && keep > 0.0) {
        return false;
    }
    let hi = Point3::new(cl.x, cl.y, top);
    for (f, b) in facets.iter().zip(boxes) {
        let (dx, dy) = ((b[0] - cl.x).max(cl.x - b[3]).max(0.0), (b[1] - cl.y).max(cl.y - b[4]).max(0.0));
        let dz = (b[2] - top).max(cl.z - b[5]).max(0.0);
        if dx * dx + dy * dy + dz * dz >= keep * keep {
            continue;
        }
        if !(dist_seg_tri(cl, hi, f) >= keep) {
            return false;
        }
    }
    !shank.is_some_and(|s| shank_hits_at(cl, pose, r, s, facets, boxes))
}

/// The by-name refusal of a blocked `column` (`Start` the plunge,
/// `Exit` the retract) on a curve that the `role` pick `p` changed.
pub(super) fn column_refusal(column: PickRole, role: PickRole, p: &PathPick, ci: usize, cl: Point3) -> GenError {
    let (what, verb) = if column == PickRole::Start { ("plunge", "descend") } else { ("retract", "leave") };
    GenError::Invalid(format!(
        "{}: the {what} column over drive curve {} at ({:.3}, {:.3}, {:.3}) crosses the part - the \
         tool cannot {verb} there; pick a point with open air above it, or clear the {} point",
        named(role, p),
        ci + 1,
        cl.x,
        cl.y,
        cl.z,
        role.word()
    ))
}

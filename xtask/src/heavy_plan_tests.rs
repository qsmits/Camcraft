//! Pins the heavy tier (#2568): the diff-to-heavy-binary mapping, also through the crate dependencies.
//! Provenance: our own build law (AGENTS.md "Heavy lanes at review"); cargo's target selectors.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use crate::plan::{self, Plan, Step};
use crate::{heavy, parse_gate};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn heavy_plan(changed: &[&str]) -> Plan {
    let changed: Vec<String> = changed.iter().map(|s| (*s).to_string()).collect();
    heavy::plan(&root(), &changed).expect("heavy plan")
}

fn names(p: &Plan) -> Vec<&str> {
    p.steps.iter().map(|s| s.name.as_str()).collect()
}

/// The `-p` packages of the release build step (the first step of a plan that has steps).
fn built(p: &Plan) -> Vec<&str> {
    let build = p.steps.first().expect("build step");
    assert_eq!(build.name, "heavy build (release --no-run)");
    build.args.windows(2).filter(|w| w[0] == "-p").map(|w| w[1].as_str()).collect()
}

fn set(items: &[&str]) -> BTreeSet<String> {
    items.iter().map(|s| (*s).to_string()).collect()
}

#[test]
fn gate_heavy_is_a_tier_with_a_base() {
    let a = |s: &str| s.split_whitespace().map(String::from).collect::<Vec<_>>();
    let g = parse_gate(&a("heavy --base origin/main --dry-run")).expect("heavy is a gate tier");
    assert_eq!((g.tier.as_str(), g.base.as_str(), g.dry), ("heavy", "origin/main", true));
    assert!(parse_gate(&a("heavy --no-snapshots")).is_err(), "--no-snapshots stays release-only");
}

#[test]
fn reach_follows_the_crate_dependencies() {
    let reach = heavy::reach(&root()).expect("manifests");
    assert_eq!(reach["app"], set(&["app", "ui"]));
    assert_eq!(reach["motion"], set(&["app", "motion", "ui"]));
    assert_eq!(reach["geometry"], set(&["app", "cl", "geometry", "motion", "post", "sim", "tools", "ui"]));
    assert_eq!(reach["kernel"], plan::ALL_CRATES.iter().map(|d| (*d).to_string()).collect::<BTreeSet<_>>());
}

#[test]
fn a_motion_source_change_reaches_the_motion_and_app_heavy_lanes() {
    let p = heavy_plan(&["crates/motion/src/finish3d.rs"]);
    assert_eq!(built(&p), ["camcraft-motion", "camcraft-app"]);
    // Exactly the release tier's steps of the two crates: same names, same commands, none of kernel.
    let want: Vec<Step> = plan::heavy_steps(&root())
        .into_iter()
        .skip(1)
        .filter(|s| !s.args.contains(&"camcraft-kernel".to_string()))
        .collect();
    assert_eq!(p.steps[1..], want[..]);
    let n = names(&p);
    for step in ["motion heavy: lib", "motion heavy: finish3d", "app heavy: lib", "app heavy: pocket3d_hsm", "app heavy: doc"] {
        assert!(n.contains(&step), "{step} missing in {n:?}");
    }
    assert!(p.notes.iter().any(|l| l.contains("app heavy: every target - depends on motion")), "{:#?}", p.notes);
}

#[test]
fn a_changed_suite_reaches_only_its_own_heavy_binary() {
    let p = heavy_plan(&["crates/app/tests/lathe_field_sim.rs"]);
    assert_eq!(names(&p), ["heavy build (release --no-run)", "app heavy: lathe_field_sim"]);
    assert_eq!(built(&p), ["camcraft-app"]);
    let run = &p.steps[1].args;
    assert!(run.contains(&"--release".to_string()) && run.last().map(String::as_str) == Some("--ignored"), "{run:?}");
    assert!(run.windows(2).any(|w| w == ["--test", "lathe_field_sim"]), "{run:?}");
}

#[test]
fn shared_test_code_reaches_the_heavy_lanes_of_its_own_crate_only() {
    let p = heavy_plan(&["crates/motion/tests/common/shape_mask.rs"]);
    assert_eq!(built(&p), ["camcraft-motion"]);
    assert!(names(&p).iter().skip(1).all(|n| n.starts_with("motion heavy: ")), "{:?}", names(&p));
    assert!(names(&p).contains(&"motion heavy: finish3d"));
}

#[test]
fn a_kernel_or_workspace_change_reaches_every_heavy_lane() {
    let all = plan::heavy_steps(&root());
    for f in ["crates/kernel/src/lib.rs", "Cargo.lock", "Cargo.toml", ".cargo/config.toml", "rust-toolchain.toml"] {
        assert_eq!(heavy_plan(&[f]).steps, all, "{f}");
    }
    // geometry sits above kernel: it reaches motion and app, not the kernel lanes.
    assert_eq!(built(&heavy_plan(&["crates/geometry/src/lib.rs"])), ["camcraft-motion", "camcraft-app"]);
}

#[test]
fn a_diff_outside_the_heavy_crates_reaches_no_heavy_lane() {
    let p = heavy_plan(&[
        "crates/ui/src/egui_app/mod.rs",
        "crates/ui/tests/gpu_render.rs",
        "crates/app/README.md",
        "docs/decisions/README.md",
        "xtask/src/heavy.rs",
        "AGENTS.md",
    ]);
    assert!(p.steps.is_empty(), "{:?}", names(&p));
    assert!(p.notes.iter().any(|l| l.contains("reaches no heavy lane")), "{:#?}", p.notes);
}

#[test]
fn a_catalog_case_reaches_the_binary_that_reads_it() {
    let changed = vec!["examples/catalog/2d/pocket/case.toml".to_string()];
    let r = heavy::reached(&root(), &changed).expect("reached");
    assert!(r.crates.is_empty(), "{r:?}");
    assert_eq!(r.targets.keys().map(String::as_str).collect::<Vec<_>>(), ["app:example_catalog"]);
    assert!(r.keeps("app", "example_catalog") && !r.keeps("app", "lib") && !r.keeps("motion", "lib"));
}

#[test]
fn an_unreadable_manifest_is_an_error_not_a_smaller_plan() {
    let empty = std::env::temp_dir().join(format!("xtask-heavy-no-manifest-{}", std::process::id()));
    let e = heavy::reach(&empty).expect_err("no manifest");
    assert!(e.contains("gate heavy: cannot read") && e.contains("Cargo.toml"), "{e}");
}

#[test]
fn runner_review_requires_heavy_lanes_without_author_results() {
    // A green quick/clippy runner does not run these release lanes. The reviewer
    // is an instruction-driven agent, so pin its common-path policy as well as reach.
    let p = heavy_plan(&["crates/motion/src/finish3d.rs"]);
    for lane in ["motion heavy: finish3d", "app heavy: pocket3d_hsm"] {
        assert!(names(&p).contains(&lane), "{lane} must require a heavy result");
    }
    let policy = include_str!("../../.claude/agents/PR-reviewer.md");
    let runner = policy.split("3. **Worktree + trial merge**").next().expect("runner path");
    assert!(runner.contains("and 4, then continue at step 5."), "runner must enter the common heavy check");
    let common = policy
        .split("5. **Heavy check on both paths**")
        .nth(1)
        .expect("heavy check outside skipped local steps")
        .split("6. **Law check against AGENTS.md**")
        .next()
        .expect("common heavy check before law check");
    for required in [
        "required after runner results and after local tests",
        "If no local worktree exists, create it and trial-merge as in step 3",
        "cargo xtask gate heavy --base <main ref> --dry-run",
        "head and all reached lanes match",
        "If the author result is absent, stale or incomplete, run",
        "cargo xtask gate heavy --base <main ref>` locally",
        "report \"heavy lanes not run\" as a blocker, never a pass",
        "A red reached\n   heavy test is a blocker",
    ] {
        assert!(common.contains(required), "common heavy check must require: {required}");
    }
}

/// One priority bullet of the reviewer's step 5: from its lead to the next bullet or the shared tail.
fn owed_bullet<'a>(common: &'a str, lead: &str) -> &'a str {
    let rest = common.split(lead).nth(1).expect("priority bullet in step 5");
    let rest = rest.split("\n   - ").next().expect("bullet");
    rest.split("\n   A red reached").next().expect("bullet end")
}

#[test]
fn an_owed_heavy_result_is_allowed_for_p3_and_lower_only() {
    // #3467: a P3-or-lower review may defer the heavy lanes to the wave run with the fixed text;
    // a P1 or P2 review keeps the strict rule (the heavy result before merge-ready).
    let policy = include_str!("../../.claude/agents/PR-reviewer.md");
    let common = policy
        .split("5. **Heavy check on both paths**")
        .nth(1)
        .and_then(|s| s.split("6. **Law check against AGENTS.md**").next())
        .expect("step 5");
    assert!(common.contains("**Owed heavy result**, #3467"), "step 5 cites the rule");
    let strict = owed_bullet(common, "   - `P1` or `P2`, or no priority label:");
    assert!(strict.contains("The heavy result\n     comes before `merge-ready`"), "{strict}");
    assert!(strict.contains("If the author result is absent, stale or incomplete, run"), "{strict}");
    assert!(strict.contains("report \"heavy lanes not run\" as a blocker, never a pass"), "{strict}");
    assert!(!strict.contains("owed to the wave run"), "P1/P2 may not defer: {strict}");
    let lenient = owed_bullet(common, "   - `P3` or lower:");
    assert!(lenient.contains("\"not run - owed to the wave run before merge\""), "{lenient}");
    assert!(lenient.contains("name each owed lane"), "{lenient}");
    assert!(lenient.contains("The PR can get `merge-ready`"), "{lenient}");
    assert!(lenient.contains("Its merge waits\n     for a green record that names its final head"), "{lenient}");

    let law = include_str!("../../AGENTS.md");
    let rule = law.split("**Owed heavy result** (#3467)").nth(1).expect("AGENTS.md rule");
    let rule = rule.split("\n- **").next().expect("rule end");
    for required in [
        "A review of a `P3` or lower PR can set\n    `merge-ready` while no heavy result covers its head",
        "\"not run - owed to the wave run before merge\"",
        "The merging\n    session does not merge the PR until a record names that head",
        "A `P1` or `P2` PR keeps the strict rule",
    ] {
        assert!(rule.contains(required), "AGENTS.md owed rule must say: {required}");
    }
    let merge = include_str!("../../.claude/skills/merge-ready/SKILL.md");
    assert!(
        merge.contains("never merge a PR whose review says \"not\n    run - owed to the wave run before merge\" until a green record names its\n    final head"),
        "the merge step refuses an owed PR without a record"
    );
}

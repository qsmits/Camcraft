// Provenance: issues #1832 and #3276 and the repository's release workflow contract.
use std::{fs, path::Path};

fn workflow() -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../.github/workflows/release.yml");
    fs::read_to_string(path).expect("read the release workflow")
}

/// One job of release.yml: from its `  <name>:` line up to the next job (a
/// line indented two spaces) or the end of the file.
fn job(name: &str) -> String {
    let workflow = workflow();
    let body = workflow
        .split_once(&format!("\n  {name}:\n"))
        .unwrap_or_else(|| panic!("release.yml has a {name} job"))
        .1;
    body.lines()
        .take_while(|line| line.is_empty() || line.starts_with("   ") || line.starts_with("  #"))
        .collect::<Vec<_>>()
        .join("\n")
}

/// The job's own `if:` condition, trimmed (None when it has none).
fn condition(name: &str) -> Option<String> {
    job(name)
        .lines()
        .find_map(|l| l.strip_prefix("    if: ").map(|c| c.trim().to_string()))
}

fn site_job() -> String {
    job("site")
}

#[test]
fn the_site_job_runs_the_site_tests_with_its_pinned_zola() {
    let job = site_job();
    let install = job.find("- name: Install pinned Zola").expect("the site job installs Zola");
    let tests = job.find("- name: Site tests").expect("the site job runs the site tests (#1832)");
    assert!(install < tests, "the site tests run after Zola is installed");
    let step = &job[tests..];
    let step = step[1..].split_once("\n      - ").map_or(step, |(s, _)| s);
    assert!(
        step.lines().any(|l| l.trim() == "run: python3 -m unittest discover -s site/tests -v"),
        "the step runs every site test, the operation reference included:\n{step}"
    );
    assert!(step.lines().any(|l| l.trim() == "ZOLA: /tmp/zola"), "the tests build with the pinned Zola:\n{step}");
    assert!(!step.contains("continue-on-error"), "a red site test fails the site job:\n{step}");
}

#[test]
fn site_only_dispatch_runs_only_site_and_pages() {
    // #3276: the input lives on the existing workflow_dispatch; no trigger is added.
    let wf = workflow();
    let on = wf.split_once("\non:\n").expect("release.yml has an on: block").1;
    let on = on.split_once("\n\n").map_or(on, |(head, _)| head);
    let triggers: Vec<&str> =
        on.lines().filter(|l| l.starts_with("  ") && !l.starts_with("   ")).map(str::trim).collect();
    assert_eq!(triggers, ["push:", "workflow_dispatch:"], "release.yml triggers:\n{on}");
    assert!(on.contains("      site_only:\n"), "workflow_dispatch has the site_only input:\n{on}");
    assert!(on.contains("        type: boolean\n        default: false"), "site_only is a boolean, off by default:\n{on}");

    // Every build job is skipped on a site_only run; aur, snap and flatpak follow linux.
    for name in ["linux", "macos", "windows", "wasm"] {
        assert_eq!(condition(name).as_deref(), Some("${{ !inputs.site_only }}"), "{name} skips on site_only");
    }
    for name in ["aur", "snap", "flatpak"] {
        assert!(job(name).contains("    needs: linux"), "{name} follows linux");
        assert_eq!(condition(name), None, "{name} has no condition of its own");
    }

    // publish stays tag-only and never runs on site_only; the site still
    // builds without the wasm job.
    assert_eq!(
        condition("publish").as_deref(),
        Some("${{ !cancelled() && !inputs.site_only && startsWith(github.ref, 'refs/tags/') }}"),
        "publish is tag-only and skips on site_only"
    );
    assert_eq!(condition("site").as_deref(), Some("${{ !cancelled() }}"));

    // pages: the tag path as before, or a site_only dispatch from a site/* branch.
    let pages = condition("pages").expect("pages has a condition");
    assert!(pages.contains("needs.site.result == 'success'"), "{pages}");
    assert!(
        pages.contains("(startsWith(github.ref, 'refs/tags/') && needs.publish.result == 'success')"),
        "the tag path still waits for publish: {pages}"
    );
    assert!(
        pages.contains(
            "(github.event_name == 'workflow_dispatch' && inputs.site_only && startsWith(github.ref, 'refs/heads/site/'))"
        ),
        "a site_only dispatch deploys only from a site/* branch: {pages}"
    );
}

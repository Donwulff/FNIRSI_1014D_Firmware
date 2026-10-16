#!/usr/bin/env python3
"""Read-only upstream inventory check. See UPSTREAM.md for source-drop review.

Exit 0: baseline unchanged; 1: upstream revision needs review; 2: check incomplete.
No downloads of archives, extraction, builds, merges, or baseline writes.
"""

import argparse
import json
from pathlib import Path
import sys
from urllib.error import URLError
from urllib.parse import quote
from urllib.request import Request, urlopen


BASELINE = Path(__file__).with_name("upstream_baseline.json")


def github_json(path):
    request = Request("https://api.github.com/repos/" + path, headers={
        "Accept": "application/vnd.github+json",
        "User-Agent": "fnirsi-upstream-check",
        "X-GitHub-Api-Version": "2022-11-28",
    })
    with urlopen(request, timeout=30) as response:
        return json.load(response)


def tree_entries(data):
    # An incomplete inventory must never look like deletions or 'no changes'.
    if data["truncated"]:
        raise ValueError("GitHub tree is truncated; inspect subtrees before reviewing")
    return {
        entry["path"]: {key: entry[key] for key in ("sha", "mode", "type")}
        for entry in data["tree"] if entry["type"] != "tree"
    }


def compare_trees(before, after):
    changes = []
    for path in sorted(before.keys() | after.keys()):
        old, new = before.get(path), after.get(path)
        if old != new:
            changes.append({
                "path": path,
                "status": "added" if old is None else "deleted" if new is None else "modified",
                "before": old,
                "after": new,
            })
    return changes


def check_repository(baseline):
    repo = baseline["repository"]
    # Omitting sha follows the default branch, including a future branch rename.
    latest = github_json(repo + "/commits?per_page=1")[0]
    head = latest["sha"]
    old = baseline["reviewed_commit"]
    result = {
        "repository": repo,
        "reviewed_commit": old,
        "head": head,
        "commit_date": latest["commit"]["committer"]["date"],
        "needs_review": head != old,
        "changes": [],
    }
    if head != old:
        before = tree_entries(github_json(repo + "/git/trees/" + old + "?recursive=1"))
        after = tree_entries(github_json(repo + "/git/trees/" + head + "?recursive=1"))
        result["changes"] = compare_trees(before, after)
        result["compare_url"] = "https://github.com/" + repo + "/compare/" + old + "..." + head
        for change in result["changes"]:
            revision = old if change["status"] == "deleted" else head
            change["url"] = "https://github.com/" + repo + "/blob/" + revision + "/" + quote(change["path"], safe="/")
    return result


def check_all(repositories):
    results = []
    for baseline in repositories:
        try:
            results.append(check_repository(baseline))
        except (OSError, URLError, ValueError, KeyError, IndexError, TypeError) as error:
            results.append({"repository": baseline["repository"], "error": str(error)})
    # A failed check takes priority even if another repository has changes.
    status = 2 if any("error" in result for result in results) else int(
        any(result["needs_review"] for result in results))
    return results, status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=BASELINE)
    parser.add_argument("--json", action="store_true", help="emit the report as JSON")
    args = parser.parse_args(argv)
    try:
        baseline = json.loads(args.baseline.read_text())
        if baseline["schema"] != 1 or not baseline["repositories"]:
            raise ValueError("expected schema 1 and a nonempty repository list")
        if not isinstance(baseline["checked_on"], str):
            raise ValueError("expected a baseline check date")
        for entry in baseline["repositories"]:
            if not entry["repository"] or not entry["reviewed_commit"]:
                raise ValueError("each repository needs a reviewed commit")
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("Invalid baseline: " + str(error), file=sys.stderr)
        return 2

    results, status = check_all(baseline["repositories"])
    if args.json:
        print(json.dumps({"baseline_date": baseline["checked_on"], "results": results}, indent=2))
    else:
        for result in results:
            if "error" in result:
                print(result["repository"] + ": CHECK FAILED: " + result["error"])
                continue
            state = "REVIEW NEEDED" if result["needs_review"] else "unchanged"
            print(f'{result["repository"]}: {state} ({result["head"][:12]}, {result["commit_date"]})')
            if result["needs_review"]:
                print("  " + result["compare_url"])
                if not result["changes"]:
                    print("  Revision changed; current file contents and modes match the baseline.")
                for change in result["changes"]:
                    print(f'  {change["status"]}: {change["path"]}')
                    print("    " + change["url"])
    return status


if __name__ == "__main__":
    sys.exit(main())

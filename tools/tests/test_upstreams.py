"""Upstream detection must not silently dismiss source drops or failed checks."""

import unittest
from unittest.mock import patch
from urllib.error import URLError

from test_review_fixes import load_tool


tool = load_tool("check_upstreams")


def tree(entries, truncated=False):
    return {"truncated": truncated, "tree": [
        {"path": path, "sha": sha, "mode": "100644", "type": "blob"}
        for path, sha in entries.items()
    ]}


def commit(sha):
    return [{"sha": sha, "commit": {"committer": {"date": "2026-10-16T00:00:00Z"}}}]


class UpstreamTests(unittest.TestCase):
    def test_replaced_archive_and_moved_source(self):
        baseline = {"repository": "example/scope", "reviewed_commit": "old"}
        before = tree({"source drop.zip": "zip1", "old/source.c": "code", "gone.c": "gone"})
        after = tree({"source drop.zip": "zip2", "new/source.c": "code"})
        with patch.object(tool, "github_json", side_effect=[commit("new"), before, after]):
            result = tool.check_repository(baseline)
        changes = {item["path"]: item for item in result["changes"]}
        self.assertTrue(result["needs_review"])
        self.assertEqual(changes["source drop.zip"]["status"], "modified")
        self.assertEqual(changes["source drop.zip"]["before"]["sha"], "zip1")
        self.assertEqual(changes["source drop.zip"]["after"]["sha"], "zip2")
        self.assertTrue(changes["source drop.zip"]["url"].endswith("/new/source%20drop.zip"))
        self.assertEqual(changes["new/source.c"]["status"], "added")
        self.assertEqual(changes["old/source.c"]["status"], "deleted")
        self.assertTrue(changes["gone.c"]["url"].endswith("/old/gone.c"))

    def test_unchanged_head_and_changed_history(self):
        baseline = {"repository": "example/scope", "reviewed_commit": "old"}
        with patch.object(tool, "github_json", return_value=commit("old")) as fetch:
            results, status = tool.check_all([baseline])
        self.assertEqual(status, 0)
        self.assertFalse(results[0]["needs_review"])
        fetch.assert_called_once()
        same = tree({"source.zip": "zip"})
        with patch.object(tool, "github_json", side_effect=[commit("new"), same, same]):
            results, status = tool.check_all([baseline])
        self.assertEqual(status, 1)
        self.assertEqual(results[0]["changes"], [])

    def test_truncation_and_network_errors_are_incomplete(self):
        repositories = [
            {"repository": "example/first", "reviewed_commit": "old"},
            {"repository": "example/second", "reviewed_commit": "old"},
        ]
        with patch.object(tool, "github_json", side_effect=[
            commit("new"), tree({}, truncated=True), URLError("offline")
        ]):
            results, status = tool.check_all(repositories)
        self.assertEqual(status, 2)
        self.assertIn("truncated", results[0]["error"])
        self.assertIn("offline", results[1]["error"])

    def test_mode_and_submodule_changes(self):
        before = tree({"build.sh": "same"})
        after = tree({"build.sh": "same"})
        after["tree"][0]["mode"] = "100755"
        after["tree"].append({"path": "vendor", "sha": "new", "mode": "160000", "type": "commit"})
        changes = tool.compare_trees(tool.tree_entries(before), tool.tree_entries(after))
        self.assertEqual([(c["path"], c["status"]) for c in changes],
                         [("build.sh", "modified"), ("vendor", "added")])


if __name__ == "__main__":
    unittest.main()

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {
    ".cc",
    ".cmake",
    ".h",
    ".html",
    ".json",
    ".md",
    ".patch",
    ".py",
    ".sh",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}
FORBIDDEN_PATHS = (
    "/" + "Users/",
    "/opt/" + "anaconda3",
)


class PortablePathTest(unittest.TestCase):
    def test_repository_text_does_not_embed_developer_paths(self):
        violations: list[str] = []
        for path in ROOT.rglob("*"):
            if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
                continue
            relative = path.relative_to(ROOT)
            if any(
                part
                in {
                    ".git",
                    ".playwright-cli",
                    ".vinext",
                    ".wrangler",
                    "__pycache__",
                    "build",
                    "dist",
                    "node_modules",
                    "output",
                    "outputs",
                }
                for part in relative.parts
            ):
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            for forbidden in FORBIDDEN_PATHS:
                if forbidden in text:
                    violations.append(f"{relative}: {forbidden}")

        self.assertEqual(violations, [], "developer-specific paths found")


if __name__ == "__main__":
    unittest.main()

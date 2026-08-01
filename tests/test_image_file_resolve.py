"""Tests for AlignedImage ImageFile path resolution."""

import os
import tempfile
import unittest

from freecad.frametools import image_point_alignment as pa


class TestResolveImageFilePath(unittest.TestCase):

    def test_absolute_path(self):
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as fh:
            path = fh.name
        try:
            self.assertEqual(pa.resolve_image_file_path(path), path)
        finally:
            os.unlink(path)

    def test_relative_to_document_dir(self):
        with tempfile.TemporaryDirectory() as tmp:
            doc_path = os.path.join(tmp, "project.FCStd")
            img_name = "photo.png"
            img_path = os.path.join(tmp, img_name)
            with open(img_path, "wb") as fh:
                fh.write(b"\x89PNG")
            with open(doc_path, "w", encoding="utf-8") as fh:
                fh.write("")

            class _Doc(object):
                FileName = doc_path

            self.assertEqual(
                pa.resolve_image_file_path(img_name, _Doc()),
                os.path.normpath(img_path))

    def test_missing_returns_empty(self):
        self.assertEqual(pa.resolve_image_file_path(""), "")
        self.assertEqual(
            pa.resolve_image_file_path("/nonexistent/image.png"), "")


if __name__ == "__main__":
    unittest.main()

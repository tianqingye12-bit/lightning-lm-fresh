"""Flat clean result paths, collision handling, and original-map protection."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from clean_map_output import create_output_directory


class CleanOutputTests(unittest.TestCase):
    def test_default_is_sibling_and_repeated_runs_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'data'/'street';source.mkdir(parents=True)
            (source/'original.pcd').write_bytes(b'original')
            first=create_output_directory(source)
            (first/'result.pcd').write_bytes(b'previous result')
            second=create_output_directory(source)
            self.assertEqual(first,source.with_name('street_clean'))
            self.assertEqual(second,source.with_name('street_clean_2'))
            self.assertTrue((second/'INCOMPLETE').is_file())
            self.assertEqual((first/'result.pcd').read_bytes(),b'previous result')
            self.assertEqual((source/'original.pcd').read_bytes(),b'original')
            self.assertFalse((source/'clean_diagnostics').exists())

    def test_explicit_directory_refuses_existing_files_and_directories(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'input';source.mkdir()
            output=create_output_directory(source,root/'chosen')
            with self.assertRaises(FileExistsError):create_output_directory(source,output)
            file=root/'file';file.write_bytes(b'keep')
            with self.assertRaises(FileExistsError):create_output_directory(source,file)
            self.assertEqual(file.read_bytes(),b'keep')

    def test_source_and_ancestor_cannot_be_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'input';source.mkdir()
            for output in (source,source.parent):
                with self.subTest(output=output),self.assertRaises(ValueError):
                    create_output_directory(source,output)


if __name__=='__main__':unittest.main()

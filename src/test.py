import os
import subprocess
import tempfile
import shutil
import pytest

@pytest.fixture()
def hostname(pytestconfig):
    return pytestconfig.getoption("hostname")

@pytest.fixture()
def timeout(pytestconfig):
    return pytestconfig.getoption("timeout")

class TestASHViewer:
    build_dir = ""
    source_path = ""
    bin_path = ""

    def setup_class(self):
        self.build_dir = tempfile.TemporaryDirectory().name
        self.source_path = os.path.dirname(os.path.realpath(__file__))
        bin_name = "ASHViewer"
        if('nt' == os.name):
            bin_name = "ASHViewer.exe"
        self.bin_path = os.path.join(self.build_dir, bin_name)

    def teardown_class(self):
        shutil.rmtree(self.build_dir, ignore_errors=True)

    def test_OMG_398_build(self):
        print("== Source directory = " + self.source_path)
        print("== build directory = " + self.build_dir)

        if not os.path.exists(self.build_dir):
            os.makedirs(self.build_dir)

        os.chdir(self.build_dir)
        if 'posix' == os.name:
            cmake_cmd = subprocess.call(args=['cmake', self.source_path])
            make_cmd = subprocess.call(args=['make'])
        elif 'nt' == os.name:
            cmake_cmd = subprocess.call(args=['cmake', self.source_path, '-G', 'NMake Makefiles','-DCMAKE_BUILD_TYPE=Release'])
            make_cmd = subprocess.call(args=['nmake'])
            win_cmd = subprocess.call(args=['windeployqt.exe',self.bin_path])
            assert (win_cmd == 0)

        assert (cmake_cmd == 0), "The cmake cmd has failed"
        assert (make_cmd == 0), "The make cmd has failed"

    def test_OMG_399_run(self, hostname, timeout):
        print("== Example binary directory = " + self.bin_path)
        print("== Sensor name = " + hostname)

        process = subprocess.Popen( [self.bin_path, hostname, '-rRd'])
        try:
            outs, errs = process.communicate(timeout=int(timeout))
        except subprocess.TimeoutExpired:
            process.kill()
            return

        assert 0, "ASHViewer finished before the timeout"

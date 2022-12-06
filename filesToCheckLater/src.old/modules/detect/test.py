import os
import subprocess
import tempfile
import shutil
import pytest

@pytest.fixture()
def hostname(pytestconfig):
    return pytestconfig.getoption("hostname")

class TestASHDetect:
    build_dir = ""
    source_path = ""
    bin_path = ""

    def setup_class(self):
        self.build_dir = tempfile.TemporaryDirectory().name
        self.source_path = os.path.dirname(os.path.realpath(__file__))
        bin_name = "ASHDetect"
        if('nt' == os.name):
            bin_name = "ASHDetect.exe"
        self.bin_path = os.path.join(self.build_dir, bin_name)

    def teardown_class(self):
        shutil.rmtree(self.build_dir, ignore_errors=True)

    def test_OMG_392_build(self):
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

        assert (cmake_cmd == 0), "The cmake cmd has failed"
        assert (make_cmd == 0), "The make cmd has failed"

    def run_cmd(self, cmd, test):

        process = subprocess.run(cmd, timeout=60,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        print(*cmd, sep = " ")
        print(process.stdout)
        print(process.stderr)

        cmd_out = process.stdout.decode()
        cmd_err = process.stderr.decode()

        message = "The test " + test + " has failed\n\n*** STDOUT ***\n\n" + cmd_out + "\n\n*** STDERR ***\n\n" + cmd_err
        assert (process.returncode == 0), message
        return cmd_out, cmd_err

    def test_OMG_393_run(self):
        print("== Example binary directory = " + self.bin_path)
        self.run_cmd([self.bin_path], "test_run")

    def test_OMG_394_find_head(self, hostname):
        cmd_out, cmd_err = self.run_cmd([self.bin_path], "test_find_head")
        assert (hostname in cmd_out), "The head " + hostname + " is not found\n\n*** STDOUT ***\n\n"+ cmd_out + "\n\n*** STDERR ***\n\n" + cmd_err

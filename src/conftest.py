def pytest_addoption(parser):
    parser.addoption("--hostname", action="store")
    parser.addoption("--timeout", action="store", default="30")
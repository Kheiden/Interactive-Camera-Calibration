import main
import pytest

class Main():

    @classmethod
    def setup_class(self):
        self.m = main.Main()

    @classmethod
    def teardown_class(self):
        pass

    #@pytest.mark.skip(reason="Not Yet Passed.")
    def test_start(self):
        output = self.m.start()
        assert output is True

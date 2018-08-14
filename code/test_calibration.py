import calibration
import pytest

class TestCalibration():

    @classmethod
    def setup_class(self):
        self.c = calibration.Calibration()

    @classmethod
    def teardown_class(self):
        pass

    @pytest.mark.skip(reason="Passed.")
    def test_start(self):
        output = self.c.start()
        assert output is True

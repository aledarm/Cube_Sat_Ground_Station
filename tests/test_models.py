import numpy as np
import pytest
from core.models import vector3, angular_error


def test_angle_wrap():
    np.testing.assert_allclose(angular_error(np.array([-179., 179., 10.]), np.array([179., -179., 5.])), [2., -2., 5.])


@pytest.mark.parametrize('value', [[1, 2], [0, 0, float('nan')], [0, 0, float('inf')], [[1], [2], [3]]])
def test_vector_rejects_bad_values(value):
    with pytest.raises(ValueError):
        vector3(value, 'test')


def test_vector_limits():
    with pytest.raises(ValueError):
        vector3([0, 0, 181], 'test', 180)

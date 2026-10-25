import numpy as np
from simulation.camera import VirtualCamera


def test_actual_pixels_detected():
    c=VirtualCamera(); rng=np.random.default_rng(1)
    image=c.render(np.array([0.,0.,30.]),np.zeros(3),True,rng)
    uv=c.detect(image)
    assert uv is not None and uv[0]<c.cx
    ref=c.pointing_reference(uv,np.array([0.,0.,30.]))
    np.testing.assert_allclose(ref,np.zeros(3),atol=.3)


def test_occluded_target_not_detected():
    c=VirtualCamera()
    image=c.render(np.zeros(3),np.zeros(3),False,np.random.default_rng(1))
    assert c.detect(image) is None


def test_offscreen_target_not_fabricated():
    c=VirtualCamera()
    image=c.render(np.array([0.,0.,120.]),np.zeros(3),True,np.random.default_rng(1))
    assert c.detect(image) is None


def test_single_centered_point_has_no_roll_reference():
    c=VirtualCamera()
    a=c.render(np.zeros(3),np.zeros(3),True,np.random.default_rng(1))
    b=c.render(np.array([25.,0.,0.]),np.zeros(3),True,np.random.default_rng(1))
    assert c.detect(a)==c.detect(b)


def test_offcentre_visual_setpoint_changes_reference():
    c=VirtualCamera(); rng=np.random.default_rng(2)
    attitude=np.array([0.,0.,20.])
    image=c.render(attitude,np.zeros(3),True,rng)
    uv=c.detect(image)
    assert uv is not None
    centered=c.pointing_reference(uv,attitude,0.,(0.,0.))
    offset=c.pointing_reference(uv,attitude,0.,(5.,-3.))
    np.testing.assert_allclose(offset-centered,[0.,-3.,-5.],atol=.3)

from gennav.utils.geometry import Point, compute_distance
from gennav.utils.samplers import UniformCircularSampler, UniformRectSampler


def test_uniform_rect_sampler():
    min_x, max_x, min_y, max_y = -5, -5, 15, 15
    sampler = UniformRectSampler(min_x, max_x, min_y, max_y)
    for _ in range(100):
        s = sampler()
        p = s.position
        assert min_x <= p.x and p.x <= max_x and min_y <= p.y and p.y <= max_y


def test_uniform_circular_sampler():
    radius = 5
    centre = Point(x=1, y=2)
    sampler = UniformCircularSampler(radius, centre)
    for _ in range(100):
        s = sampler()
        p = s.position
        assert compute_distance(p, centre) <= radius

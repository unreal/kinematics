require 'test_helper'

class TestDeltaSolver < Test::Unit::TestCase
  def test_forward
    solver = Kinematics::DeltaSolver.new(115.0, 457.3, 232.0, 112.0)
    assert_equal [0.0, 0.0, -96.859], solver.forward(0,0,0)
  end

  def test_inverse
    solver = Kinematics::DeltaSolver.new(115.0, 457.3, 232.0, 112.0)
    assert_equal [0,0,0], solver.inverse(0.0, 0.0, -96.859)
  end

  def test_inverse_again
    solver = Kinematics::DeltaSolver.new(115.0, 457.3, 232.0, 112.0)
    assert_equal [-2.718,-26.337,-7.554], solver.inverse(5.0, 5.0, -80.0)
  end
end

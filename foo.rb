class Arm
  def initialize()
    @links = []
    @joints = []
  end

  def add_link(link)
    @links << link
    add_joint if @links.length > 1
  end

  def add_joint
    @joints << Joint.new(@links[@links.length-2],@links[@links.length-1])
  end

  def last_link
    @links.last
  end
end

class Link
  def initialize(length)
    @length = length
  end
end

class Joint
  def initialize(base_link,head_link)
    @base_link = base_link
    @head_link = head_link
  end
end


a = Arm.new
a.add_link(Link.new(5))
a.add_link(Link.new(10))
puts a.inspect



require 'matrix'
def transformation_matrix(t1,t2,a1,a2)
  # convert angles to radians
  t1 = t1 * Math::PI/180
  t2 = t2 * Math::PI/180

  t12 = t1 + t2
  Matrix[
    [Math.cos(t12), -Math.sin(t12), 0, a1*Math.cos(t1) + a2*Math.cos(t12)],
    [Math.sin(t12),  Math.cos(t12), 0, a1*Math.sin(t1) + a2*Math.sin(t12)],
    [            0,              0, 1,                                  0],
    [            0,              0, 0,                                  1]
  ]
end

puts transformation_matrix(45,0,10,10)

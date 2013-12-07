# encoding: utf-8
require File.expand_path('../lib/kinematics/version', __FILE__)

Gem::Specification.new do |spec|
  spec.add_development_dependency 'test-unit'

  spec.authors                   = ["Jay Strybis"]
  spec.email                     = ['jay.strybis@gmail.com']

  spec.name                      = 'kinematics'
  spec.description               = %q{Forward and inverse kinematics library}
  spec.summary                   = spec.description
  spec.homepage                  = 'http://github.com/unreal/kinematics/'
  spec.licenses                  = ['MIT']
  spec.version                   = Kinematics::VERSION

  spec.files                     = %w(README.md Rakefile tp.gemspec)
  spec.files                    += Dir.glob("lib/**/*.rb")
  spec.files                    += Dir.glob("test/**/*")
  spec.test_files                = Dir.glob("test/**/*")

  spec.require_paths             = ['lib']
  spec.required_rubygems_version = Gem::Requirement.new('>= 1.3.6')
end


project('frc-6978-vision', 'cpp')
add_project_arguments('-Ofast', '-g3',  language : 'cpp')

cvdep = dependency('opencv4')
aprildep = dependency('apriltag')
httpinc = include_directories('submodules', 'src')

vision_sources = ['src/utils.cpp', 'src/streamer.cpp', 'src/visiondemo.cpp', 'src/posestreamerserver.cpp', 'src/posestreamerserverclient.cpp', 'src/apriltagpipeline.cpp', 'src/tagmap.cpp', 'src/notepipeline.cpp']
executable('visiondemo', vision_sources, dependencies : [cvdep, aprildep], include_directories: httpinc, install : true, install_dir: '/home/robotics/vision')
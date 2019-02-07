Name:           ros-indigo-movie-publisher
Version:        1.1.1
Release:        0%{?dist}
Summary:        ROS movie_publisher package

Group:          Development/Libraries
License:        BSD
Source0:        %{name}-%{version}.tar.gz

Requires:       ffmpeg-devel
Requires:       opencv-python
Requires:       ros-indigo-bag-tools
Requires:       ros-indigo-cv-bridge
Requires:       ros-indigo-rosbash-params >= 1.0.2
Requires:       ros-indigo-rospy
Requires:       ros-indigo-sensor-msgs
BuildRequires:  ros-indigo-catkin

%description
Node for using a video file as video topic source.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_LIBDIR="lib" \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" \
        -DCMAKE_PREFIX_PATH="/opt/ros/indigo" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/indigo

%changelog
* Thu Feb 07 2019 Martin Pecka <peckama2@fel.cvut.cz> - 1.1.1-0
- Autogenerated by Bloom

* Mon Jan 28 2019 Martin Pecka <peckama2@fel.cvut.cz> - 1.1.0-0
- Autogenerated by Bloom


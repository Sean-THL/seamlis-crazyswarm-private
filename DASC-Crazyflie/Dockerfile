FROM osrf/ros:noetic-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    wget \
    libpcl-dev \
    libusb-1.0-0-dev \
    swig \
    && rm -rf /var/lib/apt/lists/*
    

RUN echo "alias python=python3.7" >> ~/.bashrc
RUN export PATH=${PATH}:/usr/bin/python3.7
RUN /bin/bash -c "source ~/.bashrc"

RUN apt-get update && apt-get install -y --no-install-recommends \
   libboost-dev \
   libpcl-dev \
   libboost-program-options-dev \
  libusb-1.0-0-dev \
  libxcb-cursor0 \ 
  python3-pip \
  git \
  && rm -rf /var/lib/apt/lists/*


RUN apt-get install -y \
    swig libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-joy \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade pip
# Install Python packages


RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-tk \
  && rm -rf /var/lib/apt/lists/*
  
RUN pip install --ignore-installed PyYAML
RUN pip install pytest scipy matplotlib numpy cfclient 
#RUN python3 -m pip install -U jax jaxlib
RUN python3 -m pip install cvxpy
RUN pip install "cvxpy[CVXOPT,GLOP,GLPK,GUROBI,MOSEK,PDLP,SCIP,XPRESS]"
RUN pip install shapely scikit-fmm casadi do-mpc



# Source ROS environment and workspace setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/crazyswarm/ros_ws/devel/setup.bash" >> /root/.bashrc

# Set environment variable for Python version
ENV CSW_PYTHON=python3

WORKDIR /root/crazyswarm

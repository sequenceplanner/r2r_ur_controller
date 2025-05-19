FROM ghcr.io/linuxserver/baseimage-kasmvnc:ubuntujammy

ENV APPNAME="URSim"

ENV PUID=1000
ENV PGID=1000
ENV NO_FULL=TRUE

# Setup Environment
ENV DEBIAN_FRONTEND noninteractive

# Set robot model - Can be UR3, UR5 or UR10
ENV ROBOT_MODEL UR10

RUN \
    echo "**** Installing Dependencies ****" && \
    apt-get update && \
    apt-get install -qy --no-install-recommends \
    openjdk-8-jre psmisc libgcc1 libstdc++6 libc6 libcap2-bin binutils && \
    # Change java alternatives so we use openjdk8 (required by URSim) not openjdk11 that comes with guacgui
    update-alternatives --install /usr/bin/java java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java 10000

# Setup JAVA_HOME
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

WORKDIR /config

RUN \
    echo "**** Downloading URSim ****" && \
    # Download URSim Linux tar.gz
    curl https://s3-eu-west-1.amazonaws.com/ur-support-site/224811/URSim_Linux-5.17.0.128818.tar.gz -o URSim-Linux.tar.gz && \
    # Extract tarball
    tar xvzf URSim-Linux.tar.gz && \
    # Remove the tarball
    rm URSim-Linux.tar.gz && \
    # Rename the URSim folder to jus ursim
    mv  ursim-* ursim

# hack libxml to work on newer ubuntu.
RUN mkdir temp_deps
RUN cp ursim/ursim-dependencies/libxmlrpc-c-ur_1.33.14_amd64.deb temp_deps/

RUN \
  cd temp_deps && \
  ls -l && \
  ar x libxmlrpc-c-ur_1.33.14_amd64.deb && \
  mkdir temp && \
  tar -C temp -zxf control.tar.gz && \
  sed -i 's/lib32gcc1/lib32gcc-s1/' temp/control && \
  sed -i 's/>= 1:4.1.1/= 12.3.0-1ubuntu1~22.04/' temp/control && \
  cd temp && \
  ls -l && \
  tar cfz control.tar.gz * && mv control.tar.gz .. && \
  cd .. && \
  ar r libxmlrpc-c-ur_1.33.14_amd64.deb debian-binary control.tar.gz data.tar.gz && \
  mv libxmlrpc-c-ur_1.33.14_amd64.deb ../ursim/ursim-dependencies/libxmlrpc-c-ur_1.33.14_amd64.deb

# patch install script
RUN cd ursim && \
    sed -i 's/^\s*needToInstallJava$/#needToInstallJava/' install.sh &&\
    sed -i 's/^\s*installDaemonManager$/#installDaemonManager/' install.sh &&\
    sed -i 's/^\s*tty -s$/#tty -s/' install.sh &&\
    sed -i 's/^\s*pushd/#pushd/' install.sh &&\
    sed -i 's/^\s*chmod/#chmod/' install.sh &&\
    sed -i 's/^\s*popd/#popd/' install.sh &&\
    sed -i 's/lib32gcc1/lib32gcc-s1/' install.sh &&\
    sed -i 's/libcurl3/libcurl4/' install.sh &&\
    sed -i 's/apt-get -y/apt-get -y --force-yes/g' install.sh

RUN mkdir -p /config/Desktop

# run install script
RUN cd ursim && HOME=/config DEBIAN_FRONTEND="noninteractive" ./install.sh

# default installation makes it so we dont have to confirm safety on startup
COPY default.installation ursim/default.installation
RUN cp ursim/default.installation ursim/programs.UR3/
RUN cp ursim/default.installation ursim/programs.UR5/
RUN cp ursim/default.installation ursim/programs.UR10/
RUN cp ursim/default.installation ursim/programs.UR20/

RUN chown -R 1000:1000 ursim

COPY /root /

# Expose ports
# Modbus Port
EXPOSE 502
# Interface Ports
EXPOSE 29999
EXPOSE 30001-30004

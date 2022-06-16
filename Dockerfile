FROM kavrakilab/ompl

WORKDIR home

RUN apt update

RUN apt remove cmake

RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.1/cmake-3.20.1.tar.gz

RUN tar -xf cmake-3.20.1.tar.gz && cd cmake-3.20.1 \
    && ./bootstrap -- -DCMAKE_USE_OPENSSL=OFF --parallel=8 && make -j8 && make install

RUN cd .. && rm -rf cmake-3.20.1

RUN apt install git && git clone https://github.com/RedwanNewaz/RRTVF

RUN cd RRTVF && mkdir build && cd build


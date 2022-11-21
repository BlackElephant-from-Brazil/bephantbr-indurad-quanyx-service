FROM ubuntu:18.04

RUN apt-get update -y && apt-get install -y 
RUN apt update && apt upgrade -y
RUN apt install cmake -y
RUN apt-get install build-essential -y

RUN mkdir omega-ash
COPY . ./omega-ash

RUN cd omega-ash && mkdir build
RUN cd omega-ash/build && cmake ../
RUN cd omega-ash/build && make
RUN cd omega-ash/build && ./hello
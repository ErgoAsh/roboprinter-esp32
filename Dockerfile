FROM espressif/idf:release-v4.4 

WORKDIR /workspace

RUN apt update -q
RUN apt install -yq python3-pip
RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions #importlib-resources

COPY ./roboprinter_esp_entry.sh .
ENTRYPOINT [ "/workspace/roboprinter_esp_entry.sh" ]

CMD ["/bin/bash"]

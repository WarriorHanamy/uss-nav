docker build -f ./noetic/Dockerfile \
  --network=host --progress=plain \
  --build-arg http_proxy=http://172.17.0.1:7890 \
  --build-arg https_proxy=http://172.17.0.1:7890 \
  -t ros-noetic-yoloe:v2 .
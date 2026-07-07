docker build \
  -f Dockerfile \
  --network=host \
  --progress=plain \
  --build-arg http_proxy=http://192.168.101.32:7890 \
  --build-arg https_proxy=http://192.168.101.32:7890 \
  --build-arg HTTP_PROXY=http://192.168.101.32:7890 \
  --build-arg HTTPS_PROXY=http://192.168.101.32:7890 \
  -t orin-nx-jp6-noetic:v2 .
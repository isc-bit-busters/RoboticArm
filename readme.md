Each time you update code as lib, you should

docker build -f base_image/Dockerfile -t cemuelle/roboticarm-base:latest .

docker push cemuelle/roboticarm-base:latest
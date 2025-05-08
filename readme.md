Each time you update code as lib, you should

docker build -f base_image/Dockerfile -t cemuelle/roboticarm-base:latest .

docker push cemuelle/roboticarm-base:latest

-------

To build also for arm64 (Raspberry Pi 5 for example, we can use this)

Open docker desktop

docker buildx build --platform linux/amd64,linux/arm64 -f base_image/Dockerfile -t cemuelle/roboticarm-base:latest --push .
FROM cemuelle/roboticarm-base:latest
WORKDIR /app
COPY . .

CMD ["python", "pipe.py"]

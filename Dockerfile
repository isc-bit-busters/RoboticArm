FROM cemuelle/roboticarm-base:latest
WORKDIR /app
COPY . .
RUN pip install --no-cache-dir spade==3.3.3

CMD ["python", "armAgent.py"]

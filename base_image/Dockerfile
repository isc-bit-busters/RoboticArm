FROM python:3.11-slim

WORKDIR /app

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    liblapack-dev \
    libblas-dev \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Only requirements so if you change the code, it won't rebuild the libraries
COPY requirements.txt ./

RUN pip install --upgrade pip && pip install -r requirements.txt
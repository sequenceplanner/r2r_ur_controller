# Use a lightweight Python image
FROM python:3.9-slim

# Install git first
# Using --no-install-recommends reduces image size
RUN apt-get update && \
    apt-get install -y git --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Set the initial working directory where the repo will be cloned
WORKDIR /app

# Clone the repository into the /app directory
# This will create /app/micro_sp_ui
RUN git clone https://github.com/endre90/micro_sp_ui.git

# Set the working directory *inside* the cloned repository
# All subsequent commands will run relative to /app/micro_sp_ui
WORKDIR /app/micro_sp_ui

# Install the dependencies using the requirements.txt from the cloned repo
# Assumes requirements.txt is in the root of the repository
RUN pip install --no-cache-dir -r requirements.txt

# Expose Streamlit's default port (8501)
EXPOSE 8501

# Run Streamlit using the app.py from the cloned repo
# Assumes app.py is in the root of the repository
CMD ["streamlit", "run", "app.py", "--server.port=8501", "--server.address=0.0.0.0"]
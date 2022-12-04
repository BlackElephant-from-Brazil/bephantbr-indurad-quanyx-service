# Install python libs and create autorun on ubuntu
pip install -r requirements.txt
cp ./setup-omega-ash-api /etc/systemd/system
systemctl enable setup-omega-ash-api

# Build detect module
cd ./src/modules/detect
mkdir build && cd build
cmake ../
make
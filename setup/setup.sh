cd ..
pip install -r requirements.txt
cd setup
cp ./setup.service /etc/systemd/system
systemctl enable setup.service
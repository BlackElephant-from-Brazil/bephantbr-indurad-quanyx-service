# OMEGA-ASH da empresa INDURAD

## Preparando o ambiente

Comece atualizando as bibliotecas:
```
    $ sudo apt update -y
```
Coloque a senha e aguarde o termino da atualização. Em seguida rode os seguintes comandos:
```
    $ sudo apt install git -y
    $ sudo apt install python3 -y
    $ sudo apt install python3-pip -y
```

## Sequencia de comandos para setup do OMEGA-ASH
Entre como super usuário com o seguinte comando:
```
    $ sudo su
```
Depois digite a senha do usuário e siga os próximos passos:
```
    $ cd /usr
    $ git clone https://github.com/guisartori/omega-ash.git
    $ cd omega-ash/setup
    $ chmod 777 -R ./setup.sh
    $ ./setup.sh
```
Para finalizar a instalação, reinicie o computador. Quando ele reiniciar o OMEGA-ASH estará funcionando normalmente.
```
    $ reboot
``` 

## Acesse o OMEGA-ASH pelo navegador ou pela URL local
`http://127.0.0.1:5000`

<!-- para rodar o cmake:

mkdir build && cd build
cmake ../
make
./hello

para rodar o server
cd server && python serve.py -->



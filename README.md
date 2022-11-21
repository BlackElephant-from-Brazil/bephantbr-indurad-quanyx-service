# OMEGA-ASH da empresa INDURAD

## Primeiros passos
Clonar o repositório para máquina local
Abra o terminal na pasta OMEGA-ASH e rode os seguintes comandos:
```
    $ pip install -r requirements.txt
    $ rodar python ./main.py
```

<!-- para rodar o cmake:

mkdir build && cd build
cmake ../
make
./hello

para rodar o server
cd server && python serve.py -->

## Para fazer o setup
`$ chmod 777 -R ./setup.sh && ./setup.sh`




## Sequencia de comandos para setup o OMEGA-ASH
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
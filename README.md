# SERVICE da empresa INDURAD/QUANYX. Fornecido pela empresa BlackElephant do Brasil.

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

## Sequencia de comandos para setup do SERVICE

Entre como super usuário com o seguinte comando:
```
    $ sudo su
```
Depois digite a senha do usuário e siga os próximos passos:
```
    $ cd /usr
    $ git clone https://github.com/guisartori/bephantbr-indurad-quanyx-service.git
    $ cd bephantbr-indurad-quanyx-service
    $ chmod 777 -R ./setup.sh
    $ ./setup.sh
```
Para finalizar a instalação, reinicie o computador. Quando ele reiniciar o SERVICE estará funcionando normalmente.
```
    $ reboot
``` 
A BlackElephant do Brasil agradece a oportunidade...

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

Siga os passos:
```
    $ cd /usr
    $ sudo git clone https://github.com/guisartori/bephantbr-indurad-quanyx-service.git
    $ cd bephantbr-indurad-quanyx-service
    $ sudo chmod 777 -R ./setup.sh
    $ sudo ./setup.sh
```

## Configure a câmera

<!-- TODO -->


Para começar a utilizar o SERVICE normalmente basta apenas reiniciar o computador.
```
    $ reboot
``` 

## Sobre as filmagens

As gravações ficarão salvas na pasta ` /usr/bephantbr-indurad-quanyx-service/src/build ` e poderão ser acessadas normalmente pelo explorador de arquivos do Ubuntu. O SERVICE possui uma lógica de deixar somente salvo os 6 vídeos mais recentes. Isso foi criado por uma questão de performance.

A BlackElephant do Brasil agradece a oportunidade.

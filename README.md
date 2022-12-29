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

Para começar a utilizar o SERVICE normalmente basta apenas reiniciar o computador.
```
    $ reboot
``` 

## Sobre as filmagens e os arquivos PCD

As gravações ficarão salvas na pasta ` /usr/bephantbr-indurad-quanyx-service/src/build ` e poderão ser acessadas normalmente pelo explorador de arquivos do Ubuntu. O SERVICE possui uma lógica de deixar somente salvo os 6 vídeos mais recentes. Isso foi criado por uma questão de performance. 

Já os arquivos PCDs estarão salvos na pasta ` /usr/bephantbr-indurad-quanyx-service/ROS/find_objects/pcds `. Por uma questão de performance, o sistema deixa armazenado apenas os mais recentes.

## Configurações de I/O (LEDs e botões)

1 Botão na saída 7: irá reconfigurar a câmera no cabo Ethernet.

1 Botão na saída 13: irá reiniciar o serviço da câmera.

1 Botão na saída 15: serve para intercalar entre o serviço de armazenar os dados da nuvem de pontos ou gravar as imagens obtidas pela câmera.

1 LED da cor vermelha na saída 12: Alerta caso nenhum sensor tenha sido encontrado. O recomendado é remover a câmera da alimentação e conectar novamente. Se o problema persistir pressionar o botão da saída 7.

1 LED da cor vermelha na saída 11: Alerta caso a câmera tenha algum tipo de bloqueio. O recomendado é pressionar o botão da saída 13.

## Agradecimentos

A BlackElephant do Brasil agradece a oportunidade.

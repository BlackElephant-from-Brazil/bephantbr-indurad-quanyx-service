# INSTRUÇÕES PARA UTILIZAÇÃO DO OMEGA ASH

Até o momento atual o OMEGA precisa ser inicializado manualmente para economia de memória RAM. No futuro essa necessidade será resolvida.

Antes de iniciar o serviço é necessário realizar uma configuração manual. Abra um terminal e digite o seguinte comando:
```
$ ASHDetect
```
Dois retornos são possíveis:
```
Detecting sensors...
No sensor detected
```
ou 
```
Detecting sensors...
Detected sensor head : head-T-SY-20-12-162.local
```
Caso o comando não tenha encontrado nenhum sensor, verifique se a câmera está ligada e conectada e tente novamente. Verifique também no manual de instruções se a configuração foi feita corretamente na parte de INSTALLATION e NETWORKING no Guia do Usuário fornecido pela câmera.

Caso tenha encontrado o sensor, digite o seguinte comando:
```
$ ASHConfig head-T-SY-20-12-162.local -b sgm_256x128
```

Para inicializar o OMEGA primeiro inicialize o serviço do ROS disp_to_pcl. Abra um terminal e digite os seguintes comandos:
```
$ cd ~/catkin_ws
$ sudo catkin build
$ source devel/setup.bash
$ roslaunch omega_camera omega.launch
```
Deixe o terminal aberto com o serviço rodando. Este serviço é necessário para o funcionamento da coleta de dados da nuvem de pontos.

Agora precisamos rodar o serviço que irá fazer a coleta dos dados da nuvem de pontos. Abra um novo terminal e digite os seguintes comandos:
```
$ cd ~/catkin_ws
$ sudo catkin build
$ source devel/setup.bash
$ rosrun find_objects find_objects
```
Deixe o terminal aberto com o serviço rodando. Este serviço irá salvar os frames coletados da nuvem de pontos.

Abra mais um terminal. Agora iremos iniciar o SERVICE. Digite os seguintes comandos:
```
cd /usr/bephantbr-indurad-quanyx-service/src/build/
./SERVICE
```
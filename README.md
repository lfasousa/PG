#########README#########

Aplicativo desenvolvido para a monografia
apresentada ao Curso de Ciência da Computação 
do Departamento de Informática da Universidade 
Federal do Espírito Santo, como requisito parcial 
para obtenção do Grau de Bacharel em Ciência da Computação.

Animação de Avatar 3D usando Kinect
Autor: Luiz Felipe de Abreu Sousa
Orientação : Prof. Dr. Thiago Oliveira dos Santos
UFES 2017/2

Para a compilação/execução é necessária a instalação e integração de todas as bibliotecas descritas no projeto.

A utilização do C-Make com a entrada "CMakeLists.txt" é recomendada.

Para a execução do programa é necessária a utilização de um sensor de captura 3d (foi utilizado o kinect neste projeto), 
ou um conjunto de nuvens de pontos armazenadas na pasta "Input" em ordem numérica.

O programa recebe como entrada um parametro numérico, sendo:

0 - Grabber - Execução do alinhamento e mostra da entrada;
1 - Player - Player de resultados do modelo de face;
11 - Player - Player de resultados do avatar;
2 - Recorder - -Gravação da nuvem de pontos capturada na pasta "Recorder"
3 - Render - Renderização dos frames de saida na pasta "Render"

Teclas de utilização:

K - Dispara a calibragem;
T - Inicia/Finaliza a gravação;
O - Executa o algoritmo de captura de expressão para o frame atual;
D - Debug interno;
M - Carrega a mascara e modelos de face armazenados em disco;
S - Salva a nuvem de pontos atual em disco;
Teclado Numérico - Importa nuvens armazanadas em disco.




# Carrinho Wireless

> Carrinho de controle remoto utilizando o ESP32.

## Objetivo

Implementar um carrinho de controle remoto utilizando o ESP32 como microcontrolador e PlatformIO como IDE e gerenciador de projeto.

## Materiais

* ESP32 (x1)
* Motor DC (x2)
* Sensor ultrassônico de distância (x2)

## Funcionamento

Assim que o ESP32 é conectado em uma fonte de energia, verificará se já foi conectado em algum WiFi, se sim tentará conectar-se novamente a este, se não, abrirá automaticamente uma página para o usuário inserir a conexão WiFi desejada.  
Após o fim da configuração, o carrinho estará pronto para ser controlado, bastando apenas o acesso à sua página web. O projeto conta com sensores que informam ao usuário pela página web a distância do carrinho até o próximo obstáculo.

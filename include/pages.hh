#include <Arduino.h>

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-br">
	<head>
		<meta charset="UTF-8" />
		<meta name="viewport" content="width=device-width, initial-scale=1.0" />
		<title>Controle Remoto do Carrinho</title>

		<style>
			body {
				margin: 0;
				height: 100vh;
				display: flex;
				flex-direction: column;
				align-items: center;
				font-family: "Trebuchet MS", Arial, sans-serif;
				text-align: center;
				padding: 0 1.5rem;
			}

			main {
				height: 100vh;
				display: flex;
				gap: 10rem;
			}

			.sensor {
				height: 100%;
				display: flex;
				flex-direction: column;
				justify-content: center;
				align-items: center;
				gap: 1rem;
			}

			.distances {
				display: flex;
				flex-direction: column;
			}

			.distances svg {
				width: 100%;
			}

			.distance-3 svg {
				margin-top: -6rem;
			}

			.distance-2 svg {
				margin-top: -4.5rem;
			}

			.distance-1 svg {
				margin-top: -2.5rem;
			}

			.distance-6 svg {
				margin-top: -0.5rem;
			}

			.distance-7 svg {
				margin-top: -1.1rem;
			}

			.distance-8 svg {
				margin-top: -1.6rem;
			}

			.controls {
				display: flex;
				flex-direction: column;
				justify-content: center;
				align-items: center;
			}

			@media (max-width: 600px) {
				main {
					gap: 1rem;
				}
			}
		</style>
	</head>
	<body>
		<h1>Controle Remoto do Carrinho</h1>
		<main>
			<section class="sensor">
				<div class="distances">
					<div class="distance-4">
						<svg
							width="188"
							height="47"
							viewBox="0 0 188 47"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								id="front-far"
								d="M1.41423 40.6994C0.633185 39.9184 0.637892 38.6452 1.43044 37.8758C53.05 -12.2344 135.521 -12.2339 187.14 37.8758C187.933 38.6452 187.937 39.9184 187.156 40.6994L182.017 45.8387C181.236 46.6198 179.965 46.613 179.172 45.8447C131.947 0.126877 56.6231 0.126877 9.3989 45.8447C8.6053 46.613 7.33457 46.6197 6.55353 45.8387L1.41423 40.6994Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-3">
						<svg
							width="159"
							height="40"
							viewBox="0 0 159 40"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								id="front-mid-far"
								d="M1.13084 35.242C0.349787 34.4609 0.353876 33.188 1.14851 32.4208C44.7136 -9.64099 114.087 -9.64057 157.651 32.4208C158.446 33.188 158.45 34.4609 157.669 35.242L153.764 39.1473C152.982 39.9283 151.712 39.9219 150.917 39.1559C111.067 0.80572 47.733 0.80572 7.88314 39.1559C7.08726 39.9219 5.81721 39.9283 5.03616 39.1473L1.13084 35.242Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-2">
						<svg
							width="128"
							height="32"
							viewBox="0 0 128 32"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								id="front-mid"
								d="M1.42449 28.6494C0.643445 27.8684 0.646579 26.5958 1.44445 25.8319C36.4755 -7.70538 91.9749 -7.70504 127.006 25.8319C127.803 26.5958 127.807 27.8684 127.026 28.6494L124.427 31.2476C123.646 32.0286 122.377 32.0225 121.578 31.2602C89.5416 0.713163 38.9085 0.713163 6.8721 31.2602C6.0727 32.0225 4.80368 32.0286 4.02264 31.2476L1.42449 28.6494Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-1">
						<svg
							width="83"
							height="21"
							viewBox="0 0 83 21"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								id="front-close"
								d="M0.64358 18.8161C-0.137469 18.035 -0.136893 16.7633 0.669688 16.0086C23.4737 -5.3273 59.1106 -5.32709 81.9144 16.0086C82.721 16.7633 82.7216 18.035 81.9405 18.8161L81.2144 19.5422C80.4333 20.3233 79.1671 20.318 78.3582 19.5659C57.5177 0.186597 25.0665 0.186598 4.22589 19.5659C3.417 20.318 2.15077 20.3233 1.36973 19.5422L0.64358 18.8161Z"
								fill="#4E4E4E" />
						</svg>
					</div>
				</div>
				<div class="car">
					<svg
						width="78"
						height="78"
						viewBox="0 0 78 78"
						fill="none"
						xmlns="http://www.w3.org/2000/svg">
						<g clip-path="url(#clip0_3_111)">
							<path
								d="M48.75 0L29.2484 0C24.079 0 19.8898 5.74983 19.8898 10.9192V68.6397C19.8898 73.8075 24.079 78 29.2484 78H48.75C53.9178 78 58.1103 73.8091 58.1103 68.6397V10.9192C58.107 5.74983 53.9178 0 48.75 0ZM56.4701 23.53V42.8758L51.9442 43.4579V35.4874L56.4701 23.53ZM54.0952 17.8664C52.4102 24.3344 50.4151 31.9798 50.4151 31.9798H27.5816L23.8966 17.8664C23.8982 17.8664 38.6302 12.8612 54.0952 17.8664ZM26.1056 36.0098V43.4596L21.5781 42.8808V24.0508L26.1056 36.0098ZM21.5781 62.9181V45.7383L26.1056 46.3071V59.8997L21.5781 62.9181ZM24.1603 67.8006L27.8387 62.268H50.6772L54.3572 67.8006H24.1603ZM51.9442 59.3806V46.3254L56.4701 45.7366V62.4007L51.9442 59.3806Z"
								fill="#323232" />
						</g>
						<defs>
							<clipPath id="clip0_3_111">
								<rect width="78" height="78" fill="white" />
							</clipPath>
						</defs>
					</svg>
				</div>
				<div class="distances">
					<div class="distance-5">
						<svg
							width="83"
							height="21"
							viewBox="0 0 83 21"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								d="M82.1417 2.18448C82.9227 2.96553 82.9222 4.2373 82.1156 4.99195C59.3116 26.3278 23.6747 26.3276 0.870859 4.99195C0.0642781 4.2373 0.0637037 2.96553 0.844752 2.18448L1.5709 1.45833C2.35195 0.677282 3.61818 0.68252 4.42707 1.43469C25.2676 20.814 57.7188 20.814 78.5594 1.43469C79.3683 0.682519 80.6345 0.677283 81.4156 1.45833L82.1417 2.18448Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-6">
						<svg
							width="127"
							height="32"
							viewBox="0 0 127 32"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								d="M126.306 3.35138C127.087 4.13243 127.084 5.40503 126.286 6.16888C91.2553 39.7062 35.756 39.7058 0.725213 6.16888C-0.072662 5.40503 -0.0757924 4.13243 0.705256 3.35138L3.3034 0.753235C4.08445 -0.0278132 5.35346 -0.0216909 6.15287 0.740555C38.1893 31.2876 88.8223 31.2876 120.859 0.740556C121.658 -0.0216898 122.927 -0.0278124 123.708 0.753236L126.306 3.35138Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-7">
						<svg
							width="159"
							height="40"
							viewBox="0 0 159 40"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								d="M157.517 4.75761C158.299 5.53866 158.294 6.81159 157.5 7.57881C113.935 49.6406 44.5618 49.6401 0.997147 7.57881C0.202509 6.81159 0.198421 5.53866 0.979469 4.75761L4.8848 0.85228C5.66585 0.0712319 6.9359 0.07771 7.73178 0.843639C47.5817 39.1939 110.915 39.1939 150.765 0.843639C151.561 0.07771 152.831 0.0712334 153.612 0.852282L157.517 4.75761Z"
								fill="#4E4E4E" />
						</svg>
					</div>
					<div class="distance-8">
						<svg
							width="188"
							height="47"
							viewBox="0 0 188 47"
							fill="none"
							xmlns="http://www.w3.org/2000/svg">
							<path
								d="M187.156 6.30059C187.937 7.08164 187.933 8.35479 187.14 9.12417C135.52 59.2344 53.0495 59.2339 1.43044 9.12417C0.637894 8.3548 0.633186 7.08164 1.41423 6.30059L6.55353 1.1613C7.33458 0.380249 8.6053 0.386964 9.3989 1.15525C56.6231 46.8731 131.947 46.8731 179.172 1.15526C179.965 0.386967 181.236 0.380251 182.017 1.1613L187.156 6.30059Z"
								fill="#4E4E4E" />
						</svg>
					</div>
				</div>
			</section>
			<section class="controls">
				<div id="joystck"></div>
				<input type="range" id="vel" list="tickmarks" min="30" max="100" value="100" />
				<datalist id="tickmarks">
					<option value="30" label="30%"></option>
					<option value="40"></option>
					<option value="50" label="50%"></option>
					<option value="60"></option>
					<option value="70"></option>
					<option value="80"></option>
					<option value="90"></option>
					<option value="100" label="100%"></option>
				</datalist>
			</section>
		</main>

		<!-- prettier-ignore -->
		<script defer>
            let StickStatus={xPosition:0,yPosition:0,x:0,y:0,cardinalDirection:"C"};var JoyStick=function(t,e,i){var o=void 0===(e=e||{}).title?"joystick":e.title,n=void 0===e.width?0:e.width,a=void 0===e.height?0:e.height,r=void 0===e.internalFillColor?"#00AA00":e.internalFillColor,c=void 0===e.internalLineWidth?2:e.internalLineWidth,s=void 0===e.internalStrokeColor?"#003300":e.internalStrokeColor,d=void 0===e.externalLineWidth?2:e.externalLineWidth,u=void 0===e.externalStrokeColor?"#008000":e.externalStrokeColor,h=void 0===e.autoReturnToCenter||e.autoReturnToCenter;i=i||function(t){};var S=document.getElementById(t);S.style.touchAction="none";var f=document.createElement("canvas");f.id=o,0===n&&(n=S.clientWidth),0===a&&(a=S.clientHeight),f.width=n,f.height=a,S.appendChild(f);var l=f.getContext("2d"),k=0,g=2*Math.PI,x=(f.width-(f.width/2+10))/2,v=x+5,P=x+30,m=f.width/2,C=f.height/2,p=f.width/10,y=-1*p,w=f.height/10,L=-1*w,F=m,E=C;function W(){l.beginPath(),l.arc(m,C,P,0,g,!1),l.lineWidth=d,l.strokeStyle=u,l.stroke()}function T(){l.beginPath(),F<x&&(F=v),F+x>f.width&&(F=f.width-v),E<x&&(E=v),E+x>f.height&&(E=f.height-v),l.arc(F,E,x,0,g,!1);var t=l.createRadialGradient(m,C,5,m,C,200);t.addColorStop(0,r),t.addColorStop(1,s),l.fillStyle=t,l.fill(),l.lineWidth=c,l.strokeStyle=s,l.stroke()}function D(){let t="",e=F-m,i=E-C;return i>=L&&i<=w&&(t="C"),i<L&&(t="N"),i>w&&(t="S"),e<y&&("C"===t?t="W":t+="W"),e>p&&("C"===t?t="E":t+="E"),t}"ontouchstart"in document.documentElement?(f.addEventListener("touchstart",function(t){k=1},!1),document.addEventListener("touchmove",function(t){1===k&&t.targetTouches[0].target===f&&(F=t.targetTouches[0].pageX,E=t.targetTouches[0].pageY,"BODY"===f.offsetParent.tagName.toUpperCase()?(F-=f.offsetLeft,E-=f.offsetTop):(F-=f.offsetParent.offsetLeft,E-=f.offsetParent.offsetTop),l.clearRect(0,0,f.width,f.height),W(),T(),StickStatus.xPosition=F,StickStatus.yPosition=E,StickStatus.x=((F-m)/v*100).toFixed(),StickStatus.y=((E-C)/v*100*-1).toFixed(),StickStatus.cardinalDirection=D(),i(StickStatus))},!1),document.addEventListener("touchend",function(t){k=0,h&&(F=m,E=C);l.clearRect(0,0,f.width,f.height),W(),T(),StickStatus.xPosition=F,StickStatus.yPosition=E,StickStatus.x=((F-m)/v*100).toFixed(),StickStatus.y=((E-C)/v*100*-1).toFixed(),StickStatus.cardinalDirection=D(),i(StickStatus)},!1)):(f.addEventListener("mousedown",function(t){k=1},!1),document.addEventListener("mousemove",function(t){1===k&&(F=t.pageX,E=t.pageY,"BODY"===f.offsetParent.tagName.toUpperCase()?(F-=f.offsetLeft,E-=f.offsetTop):(F-=f.offsetParent.offsetLeft,E-=f.offsetParent.offsetTop),l.clearRect(0,0,f.width,f.height),W(),T(),StickStatus.xPosition=F,StickStatus.yPosition=E,StickStatus.x=((F-m)/v*100).toFixed(),StickStatus.y=((E-C)/v*100*-1).toFixed(),StickStatus.cardinalDirection=D(),i(StickStatus))},!1),document.addEventListener("mouseup",function(t){k=0,h&&(F=m,E=C);l.clearRect(0,0,f.width,f.height),W(),T(),StickStatus.xPosition=F,StickStatus.yPosition=E,StickStatus.x=((F-m)/v*100).toFixed(),StickStatus.y=((E-C)/v*100*-1).toFixed(),StickStatus.cardinalDirection=D(),i(StickStatus)},!1)),W(),T(),this.GetWidth=function(){return f.width},this.GetHeight=function(){return f.height},this.GetPosX=function(){return F},this.GetPosY=function(){return E},this.GetX=function(){return((F-m)/v*100).toFixed()},this.GetY=function(){return((E-C)/v*100*-1).toFixed()},this.GetDir=function(){return D()}};
        </script>

		<script defer>
			const joystick = new JoyStick("joystck", {
				width: 200,
				height: 200,
				internalFillColor: "#007AFF",
				internalStrokeColor: "#D0D0D0",
				externalStrokeColor: "#007AFF",
			});

			const socket = new WebSocket(`ws://${window.location.host}/ws`);
			socket.onopen = () => {
				console.log("Conexão WebSocket aberta com sucesso.");
			};

			socket.onmessage = (event) => {
				const data = JSON.parse(event.data);

				if (data.frontSensor <= 12) {
					document.getElementById("front-close").style.fill = "red";
					document.getElementById("front-mid").style.fill = "#4E4E4E";
					document.getElementById("front-mid-far").style.fill = "#4E4E4E";
					document.getElementById("front-far").style.fill = "#4E4E4E";
				} else if (data.frontSensor > 12 && data.frontSensor <= 20) {
					document.getElementById("front-close").style.fill = "#4E4E4E";
					document.getElementById("front-mid").style.fill = "yellow";
					document.getElementById("front-mid-far").style.fill = "#4E4E4E";
					document.getElementById("front-far").style.fill = "#4E4E4E";
				} else if (data.frontSensor > 20 && data.frontSensor <= 28) {
					document.getElementById("front-close").style.fill = "#4E4E4E";
					document.getElementById("front-mid").style.fill = "#4E4E4E";
					document.getElementById("front-mid-far").style.fill = "yellow";
					document.getElementById("front-far").style.fill = "#4E4E4E";
				} else if (data.frontSensor > 28 && data.frontSensor <= 36) {
					document.getElementById("front-close").style.fill = "#4E4E4E";
					document.getElementById("front-mid").style.fill = "#4E4E4E";
					document.getElementById("front-mid-far").style.fill = "#4E4E4E";
					document.getElementById("front-far").style.fill = "yellow";
				} else {
					document.getElementById("front-close").style.fill = "#4E4E4E";
					document.getElementById("front-mid").style.fill = "#4E4E4E";
					document.getElementById("front-mid-far").style.fill = "#4E4E4E";
					document.getElementById("front-far").style.fill = "#4E4E4E";
				}
			};

			socket.onclose = () => {
				console.info("A conexão WebSocket foi encerrada.");
			};

			socket.onerror = (error) => {
				console.error(`Houve um erro com a conexão WebSocket: ${error.error}`);
			};

			const velInput = document.getElementById("vel");
			velInput.onchange = (event) => {
				const vel = Number(event.target.value);
				fetch(`/changeVel?vel=${vel}`);
			};

			let lastDir;
			setInterval(() => {
				const dir = joystick.GetDir();
				if (!socket.readyState || lastDir == dir) return;
				lastDir = dir;

				if (dir === "N" || dir === "NE" || dir === "NW") {
					socket.send("forward");
				} else if (dir === "S" || dir === "SE" || dir === "SW") {
					socket.send("backward");
				} else if (dir === "W") {
					socket.send("left");
				} else if (dir === "E") {
					socket.send("right");
				} else {
					socket.send("stop");
				}
			}, 20);
		</script>
	</body>
</html>
)rawliteral";

const char CONSOLE_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-br">
	<head>
		<meta charset="UTF-8" />
		<meta name="viewport" content="width=device-width, initial-scale=1.0" />
		<title>CONSOLE - Controle Remoto do Carrinho</title>

		<style>
			body {
				margin: 0;
				height: 100vh;
				display: flex;
				flex-direction: column;
				align-items: center;
				font-family: "Trebuchet MS", Arial, sans-serif;
				text-align: center;
				padding: 0 1.5rem;
			}

			main {
				height: 100vh;
				display: flex;
				gap: 10rem;
			}

			@media (max-width: 600px) {
				main {
					gap: 1rem;
				}
			}

			#reset {
				width: 100%;
				padding: 1rem 0;
				background-color: rgb(212, 52, 52);
				border: none;
				border-radius: 0.2rem;
				cursor: pointer;
				font-size: large;
			}

			#reset:hover {
				background-color: rgb(190, 48, 48);
			}

			#reset:active {
				background-color: rgb(170, 47, 47);
			}

			#espnow {
				width: 100%;
				padding: 1rem 0;
				background-color: rgb(52, 103, 212);
				border: none;
				border-radius: 0.2rem;
				cursor: pointer;
				font-size: large;
			}

			#espnow:hover {
				background-color: rgb(49, 95, 194);
			}

			#espnow:active {
				background-color: rgb(43, 81, 163);
			}

			#stopmotors {
				width: 100%;
				padding: 1rem 0;
				background-color: rgb(228, 193, 38);
				border: none;
				border-radius: 0.2rem;
				cursor: pointer;
				font-size: large;
			}

			#stopmotors:hover {
				background-color: rgb(204, 173, 35);
			}

			#stopmotors:active {
				background-color: rgb(182, 154, 30);
			}

			#alertbox {
				display: none;
				position: fixed;
				top: 50%;
				background-color: rgb(240, 240, 68);
				padding: 1rem;
				border: 1px solid rgb(173, 173, 33);
				font-size: larger;
			}
		</style>
	</head>
	<body>
		<h1>Painel de controle do Carrinho</h1>
		<main>
			<div>
				<h2>Comunicação por WiFi: <span id="wifiMenuSelected"></span></h2>
				<h2>Status do WiFi: <span id="wifiStatus"></span></h2>
				<h2>Velocidade máxima dos motores: <span id="motorsVelocity">0</span></h2>
				<h2>Clientes conectados: <span id="clients">0</span></h2>
				<h2>Perda de pacotes: <span id="packetloss">Não disponível</span></h2>
			</div>
			<div>
				<h2>RAM total: <span id="espHeap">0</span></h2>
				<h2>RAM livre: <span id="espFreeHeap">0</span></h2>
				<h2>RAM usada: <span id="espUsedHeap">0</span></h2>
				<h2>IP: <span id="ip"></span></h2>
				<h2>MAC: <span id="mac"></span></h2>
			</div>
			<div style="display: flex; flex-direction: column; gap: 1rem">
				<h2>Sistema irá resetar: <span id="erasingMemory"></span></h2>
				<button id="reset">Iniciar reset</button>
				<button id="espnow">Trocar para ESP-NOW</button>
				<button id="stopmotors">Parar motores</button>
			</div>
		</main>
		<div id="alertbox"></div>

		<script>
			const WIFI_STATUS_ENUM = {
				255: "Compatível",
				0: "Em espera",
				1: "SSID indisponível",
				2: "Scan completo",
				3: "Conectado",
				4: "Conexão falhou",
				5: "Conexão perdida",
				6: "Senha incorreta",
				7: "Desconectado",
			};

			function formatBytes(bytes, decimalPoint) {
				if (bytes == 0) return "0 Bytes";
				var k = 1000,
					dm = decimalPoint || 2,
					sizes = ["Bytes", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"],
					i = Math.floor(Math.log(bytes) / Math.log(k));
				return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + " " + sizes[i];
			}

			let lastPacketMS = new Date();
			let diffAverage = 0;
			setInterval(() => {
				const packetMS = new Date();
				let diff = Math.abs(lastPacketMS - packetMS);
				diffAverage = (diffAverage + diff) / 2;
				const percentage = (diffAverage * 100) / 1000;

				const packetloss = document.getElementById("packetloss");
				packetloss.innerText = `${new Intl.NumberFormat("pt-BR", {
					style: "decimal",
					maximumFractionDigits: 1,
				}).format(percentage)}% (${new Intl.NumberFormat("pt-BR", {
					style: "decimal",
					maximumFractionDigits: 1,
				}).format(diffAverage)})`;

				if (diffAverage < 200) {
					packetloss.style.color = "green";
				}
				if (diffAverage > 200 && diffAverage < 800) {
					packetloss.style.color = "yellow";
				}
				if (diffAverage > 800) {
					packetloss.style.color = "red";
				}
			}, 100);
		</script>

		<script defer>
			const socket = new WebSocket(`ws://${window.location.host}/ws`);
			const alertbox = document.getElementById("alertbox");
			socket.onopen = () => {
				console.log("Conexão WebSocket aberta com sucesso.");
				alertbox.style.display = "none";
			};

			socket.onmessage = (event) => {
				lastPacketMS = new Date();
				const data = JSON.parse(event.data);

				const wifiMenuSelected = data.wifiMenuSelected;
				const wifiStatus = data.wifiStatus;
				const motorsVelocity = data.motorsVelocity;
				const motorsVelocityPercentage = new Intl.NumberFormat("pt-BR", {
					style: "decimal",
					maximumFractionDigits: 1,
				}).format((motorsVelocity * 100) / 255);
				const clients = data.clients;
				const espHeap = data.espHeap;
				const espFreeHeap = data.espFreeHeap;
				const espUsedHeap = espHeap - espFreeHeap;
				const ip = data.ip;
				const mac = data.mac;
				const erasingMemory = data.erasingMemory;

				document.getElementById("wifiMenuSelected").innerText = wifiMenuSelected
					? "Ativada"
					: "Desativada";
				document.getElementById(
					"motorsVelocity"
				).innerText = `${motorsVelocity} (${motorsVelocityPercentage}%)`;
				document.getElementById("wifiStatus").innerText = WIFI_STATUS_ENUM[wifiStatus];
				document.getElementById("clients").innerText = clients;
				document.getElementById("espHeap").innerText = formatBytes(espHeap);
				document.getElementById("espFreeHeap").innerText = formatBytes(espFreeHeap);
				document.getElementById("espUsedHeap").innerText = formatBytes(espUsedHeap);
				document.getElementById("ip").innerText = ip;
				document.getElementById("mac").innerText = mac;

				const erasing = document.getElementById("erasingMemory");
				erasing.innerText = erasingMemory ? "SIM" : "Não";
				erasing.style.color = erasingMemory ? "red" : "green";
				erasing.style.scale = "2";
			};

			socket.onclose = () => {
				console.info("A conexão WebSocket foi encerrada.");
				alertbox.style.display = "inline";
				alertbox.innerText = "Conexão encerrada!";
			};

			socket.onerror = (error) => {
				console.error(`Houve um erro com a conexão WebSocket: ${error.error}`);
				alertbox.style.display = "inline";
				alertbox.innerText = `Falha na conexão: ${error.error}`;
			};

			document.getElementById("reset").onclick = () => {
				socket.send("reset");
			};

			document.getElementById("espnow").onclick = () => {
				socket.send("enmenu");
			};

			document.getElementById("stopmotors").onclick = () => {
				socket.send("stop");
			};
		</script>
	</body>
</html>
)rawliteral";
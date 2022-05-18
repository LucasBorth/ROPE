var area = document.getElementById("area")
var img = document.getElementById("seta")
var joy = document.getElementById("joy_area")
var canvas = area.getContext("2d")
var grauX = document.getElementById("grauX")
var grauY = document.getElementById("grauY")

var coordenada_canvas = area.getBoundingClientRect()

var coordenada_canvas_left = coordenada_canvas["left"]
var coordenada_canvas_right = coordenada_canvas["right"] - 50 //Menos 50 do Select
var coordenada_canvas_top = coordenada_canvas["top"]
var coordenada_canvas_bottom = coordenada_canvas["bottom"] - 50 //Menos 50 do Select

var centro_x = (coordenada_canvas_left + coordenada_canvas_right) / 2
var centro_y = (coordenada_canvas_top + coordenada_canvas_bottom) / 2

area.addEventListener("touchstart", evento_touch)
area.addEventListener("touchend", fim_touch)
area.addEventListener("touchmove", evento_touch)

var taxa = 40 // Taxa de Erro do Joystick

function evento_touch(entrada){
    // console.log("-- Touch --")

    var x = Math.round(Number(entrada.touches[0]["clientX"]))
    var y = Math.round(Number(entrada.touches[0]["clientY"]))

    // console.log(entrada.touches[0]) //Dados do Objeto no console

    // Seleção do Joystick

    if (x >= coordenada_canvas_left && x <= coordenada_canvas_right && y >= coordenada_canvas_top && y<= coordenada_canvas_bottom){
        grauX.innerHTML = x + "º"
        grauY.innerHTML = y + "º"
        
        canvas.fillStyle = "#66ff00"
        canvas.fillRect(0, 0, 200, 200) //Eixo X, Eixo Y, Largura, Altura
        
        joy.innerHTML = ""
        var select = document.createElement("div")
        select.setAttribute("id", "select")
        select.style.left = String(x) + "px"
        select.style.top = String(y) + "px"
        joy.appendChild(select)

        //Visualização do centro -  Ignorar
        var selectc = document.createElement("div")
        selectc.setAttribute("id", "selectc")
        selectc.style.left = centro_x + "px"
        selectc.style.top = centro_y + "px"
        joy.appendChild(selectc)

        //Informação para o Arduino!!! ------- ARDUINO UNO CODE -------
        

        if (y < centro_y - taxa && y < x){
            //Frente
            img.setAttribute("src", "src/seta.png")
            img.style.transform = "rotate(270deg)"

        }
        if (y > centro_y + taxa && y > x){
            //Tráz
            img.setAttribute("src", "src/seta.png")
            img.style.transform = "rotate(90deg)"

        } 
        if (x < centro_x - taxa && x < y){
            //Esquerda
            img.setAttribute("src", "src/seta.png")
            img.style.transform = "rotate(180deg)"

        } 
        if (x > centro_x + taxa && x > y){
            //Direita
            img.setAttribute("src", "src/seta.png")
            img.style.transform = "rotate(0deg)"
        }


    } else {
        //Fora da área de Jostick Canvas

        canvas.fillStyle = "#ff0000"
        canvas.fillRect(0, 0, 200, 200) //Eixo X, Eixo Y, Largura, Altura
    }
    
}

function fim_touch(entrada){
    // console.log("-- Fim de Touch --")

    canvas.fillStyle = "#0077ff"
    canvas.fillRect(0, 0, 200, 200)

    grauX.innerHTML = "0º"
    grauY.innerHTML = "0º"

    img.setAttribute("src", "src/imovel.png")

    joy.innerHTML = ""
}
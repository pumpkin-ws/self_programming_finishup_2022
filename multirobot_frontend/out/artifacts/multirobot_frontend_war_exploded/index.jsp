<html>
<head>
    <title>MultiRobot Control</title>
    <script src='https://ajax.aspnetcdn.com/ajax/jQuery/jquery-3.2.1.min.js'></script>
    <style>
        #box {
            position: relative;
            margin-bottom: 20px;
        }
        .left {
            position: absolute;
            width: 150px;
            height: 900px;
            border-style: solid;
            border-color: darkturquoise;
        }
        .right {
            height: 900px;
            margin-left: 150px;
            border-style: solid;
            border-color: aqua;
        }
        .butt_elem{
            margin: 20px;
            font-size: large;
        }
    </style>
</head>
<body>
<div style="border-style: dashed; border-color: blue;"; >
    <h1 style="text-align: center;">MetaRobot Controller</h1>
    <h2 style="text-align: center; font-size: 20px;">This application can control multiple robots through cloud services</h2>
</div>
<div>
    <button id="testbut">Test Robot Cloud</button>
</div>
<script>
    $("#testbut").click(function (){
        console.log("Test Robot cloud detected");
        $.post("RobotCloud");
    })
</script>
</body>
</html>

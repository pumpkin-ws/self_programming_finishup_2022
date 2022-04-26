$("#create").click(
  function(){
      console.log("Refresh main_frame to avatar creation");
      $("#main_frame").attr("src", "web_pages/create_avatar.html");
  }
);

$("#chat").click(
    function(){
        console.log("Switching to the chat interface");
        $("#main_frame").attr("src", "web_pages/chat_bot.html");
    }
)

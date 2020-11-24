let deviceStatus;
let sensorData;

function getDeviceStatus() {
  axios.get("/device", {responseType : 'json'})
  .then((res) => {
    deviceStatus = res.data;
    updatePageContents();
  }, (err) => {
      console.log(err);
  });
} // getDeviceStatus

function getSensorData(sensorId) {
  axios.get(`/sensor?id=${sensorId}`, {responseType : 'json'})
  .then((res) => {
    sensorData = res.data;
    updatePageContents();
  }, (err) => {
      console.log(err);
  });
} // getSensorData

function updatePageContents()
{
  let spanCurrentTime = document.getElementById("currentTime");
  let spanCurrentSensorData = document.getElementById("currentSensorData");
  let spanWifiSSID = document.getElementById("wifiSSID");
  let spanWifiRSSI = document.getElementById("wifiRSSI");

  // formatting for the shown current date/time
  function dateString (date) {
    var weekDays = ["Zondag","Maandag", "Dinsdag", "Woensdag", "Donderdag","Vrijdag", "Zaterdag"];
    //date = new Date();
    return `${weekDays[date.getDay()]} ${date.getDate()}/${date.getMonth()+1}/${date.getFullYear()}
                ${date.getHours().toString().padStart(2,"0")}:${date.getMinutes().toString().padStart(2,"0")}:${date.getSeconds().toString().padStart(2,"0")}`;
  }  
  var deviceTime = new Date(1000*deviceStatus.time); // device works with seconds since 1970, not ms
  spanCurrentTime.innerHTML = dateString(deviceTime);
  spanWifiSSID.innerHTML = deviceStatus.wifiSSID;
  spanWifiRSSI.innerHTML = deviceStatus.wifiRSSI;
  spanCurrentSensorData.innerHTML = '<br>';
  if (sensorData){
    if (sensorData.solarValue != undefined) spanCurrentSensorData.innerHTML+= `solar : ${(sensorData.solarValue*5.0/1023).toFixed(2)}V <br>`;
    if (sensorData.batteryValue != undefined) spanCurrentSensorData.innerHTML+= `battery : ${(sensorData.batteryValue/1000).toFixed(2)}V <br>`; // na de update met de readVcc trick
    if (sensorData.packetsOK != undefined) spanCurrentSensorData.innerHTML+= `packetsOK : ${sensorData.packetsOK} <br>`;
    if (sensorData.packetsNOK != undefined) spanCurrentSensorData.innerHTML+=  `packetsNOK : ${sensorData.packetsNOK} <br>`;
  }
} // updatePageContents

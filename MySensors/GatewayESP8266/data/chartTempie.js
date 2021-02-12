function loadChartTempie() {

  let deviceDataItems = 5; // device config, we kunnen hier nog de data types voor parseCSV, labels,axes etc voor de line graphs toevoegen TODO
  let deviceData = []; // [{dt:<epoch_in_ms>, y:[array_of_data_items]}]
  let chartData  = []; // [[{t:<>,y:<>}]] : chartData[0] = line graph 0, ...
  let chart;
  let ctx, cfg;
  
  ctx = document.getElementById('myChart').getContext('2d');
  ctx.canvas.width = window.innerWidth;
  ctx.canvas.height = 450;
  
  cfg = {
    data: {
      datasets: [
        {
          label: 'last seen',
          backgroundColor: 'yellow',
          yAxisID: 'ms-axis',
          borderColor: 'yellow',
          data : chartData[1],
          type: 'line',
        },
        {
        label: 'BMP280 hPa',
        backgroundColor: 'green',
        yAxisID: 'pressure-axis',
        borderColor: 'green',
        data : chartData[2],
        type: 'line',
      },
      {
        label: 'BMP280 °C',
        yAxisID: 'temperature-axis',
        backgroundColor: 'lightblue',
        borderColor: 'lightblue',
        data : chartData[3],
        type: 'line',
      },
      {
        label: 'Si7021 %',
        yAxisID: 'percentage-axis',
        backgroundColor: 'orange',
        borderColor: 'orange',
        data : chartData[4],
        type: 'line',
      },
      {
        label: 'Si7021 °C',
        yAxisID: 'temperature-axis',
        backgroundColor: 'blue',
        borderColor: 'blue',
        data : chartData[5],
        type: 'line',
      },
    ]
    },
    options: {
      animation : false,
      hover: {
        animationDuration: 0 // duration of animations when hovering an item
      },
      responsiveAnimationDuration: 0, // animation duration after a resize
      elements: { // dit geldt voor alle datasets, tenzij per dataset overridden
        line : {
          tension : 0, // disable bezier curves
          fill : false,
          stepped : false,
          borderWidth : 2,
          borderDash: [],
        },
        point  : {
          radius : 0, // 0 = geen bolleke op het meetpunt, maar dan wel showLine nodig, anders zie je niets
          hitRadius : 10,
        }
      },
      showLine : true,
      parsing : false,
      spanGaps : true,
  
      scales: {
        xAxes: [{
          type: 'time',
          distribution: 'linear', // 'series'
          offset: true,
          time : {
            // minUnit : 'minute', // limit zoom-in resolution
            displayFormats : {
              day : 'DD/MM',
              month : 'MMM YYYY',
              hour : 'HH:mm',
              minute : 'HH:mm',
              second : 'HH:mm:ss',
              millisecond : 'HH:mm:ss',
            },
          },
          ticks: {
            major: {
              enabled: true,
              fontStyle: 'bold'
            },
            source: 'auto', // 'data'
          },
        }],
        yAxes: [{
          id: 'temperature-axis',
          type: 'linear',
          position: 'left', // 'right', 'left' = default
          gridLines: {
            drawBorder: false
          },
          scaleLabel: {
            display: true,
            labelString: 'temperature (°C)',
            fontColor : 'rgb(0, 99, 132)',
          }
        },
        {
          id: 'percentage-axis',
          type: 'linear', // default wss
          position : 'left', 
          gridLines: {
            drawBorder: false // false : wat doet dat?
          },
          scaleLabel: {
            display: true,
            labelString: 'percentage (%)',
            fontColor : 'orange',
          },
          ticks : {
            min : 0,
          }
        },
        {
          id: 'ms-axis',
          type: 'linear', // default wss
          position : 'right', 
          gridLines: {
            drawBorder: false // false : wat doet dat?
          },
          scaleLabel: {
            display: true,
            labelString: 'lastSeen (ms)',
            fontColor : 'yellow',
          },
          ticks : {
            min : 0,
          }
        },
        {
          id: 'pressure-axis',
          type: 'linear', // default wss
          position : 'right', // 'right','left' = default
          gridLines: {
            drawBorder: false // false : wat doet dat?
          },
          scaleLabel: {
            display: true,
            labelString: 'pressure (hPa)',
            fontColor : 'green',
          },
          ticks : { // in chartjs v2 zit min/max onder ticks object <> v3, voorlopig beta
            min : 970,
            max : 1045,
          }
        }],
      },
      plugins: {
        zoom: {
          pan: { // Container for pan options
            enabled: true, // Boolean to enable panning
            // Panning directions. Remove the appropriate direction to disable
            // Eg. 'y' would only allow panning in the y direction
            // A function that is called as the user is panning and returns the
            // available directions can also be used:
            //   mode: function({ chart }) {
            //     return 'xy';
            //   },
            mode: 'x',
            rangeMin: { // Format of min pan range depends on scale type
              x: null,
              y: null
            },
            rangeMax: { // Format of max pan range depends on scale type
              x: null,
              y: null
            },
            onPanComplete: onPanZoom,
          },
          zoom: {  // Container for zoom options
            enabled: true, // Boolean to enable zooming
            drag: false, // Enable drag-to-zoom behavior
            // Drag-to-zoom effect can be customized
            // drag: {
            // 	 borderColor: 'rgba(225,225,225,0.3)'
            // 	 borderWidth: 5,
            // 	 backgroundColor: 'rgb(225,225,225)',
            // 	 animationDuration: 0
            // },
  
            // Zooming directions. Remove the appropriate direction to disable
            // Eg. 'y' would only allow zooming in the y direction
            // A function that is called as the user is zooming and returns the
            // available directions can also be used:
            //   mode: function({ chart }) {
            //     return 'xy';
            //   },
            mode: 'x',
            rangeMin: { // Format of min zoom range depends on scale type
              x: null,
              y: null
            },
            rangeMax: {// Format of max zoom range depends on scale type
              x: null,
              y: null
            },
            speed: 0.5, // (percentage of zoom on a wheel event)
            onZoomComplete : onPanZoom,
          } // zoom
        } // plugin-zoom
      } // plugin
    } // options
  };
  chart = new Chart(ctx, cfg);
  loadCSV();
  
  function onPanZoom({chart}) {
  // filter device data
  const filterDataInView = (item) => {
    return (item.t > chart.scales["x-axis-0"].min && item.t < chart.scales["x-axis-0"].max);
  };
  let filteredData = deviceData.filter(filterDataInView);
  let xPixels = chart.canvas.width;
  //console.log(`onZoom : data.length = ${chartData.length} , xPixels = ${xPixels}`);
  
  for (let i=0;i<deviceDataItems;i++) {
    // als we nog altijd meer data dan pixels hebben doen we aggregate per uur
    // TODO : voor lange periodes nog een aggregate per dag toe te voegen
    if (filteredData.length > xPixels) {
      chartData[i] = aggregateDataAvgPerHour(extract(filteredData,i));
      //console.log(`onZoom - aggregated chart data, data length is now :${chartData[i].length}`)
    }
    else {
      chartData[i] = extract(filteredData,i);
    }
    cfg.data.datasets[i].data = chartData[i];
  }
  chart.update();
  } // onPanZoom
  
  // IN : data : [{t:epoch_in_ms, y: [array_of_data_items]}]
  // OUT : [{t:in.t, y:in.y[idx]}]
  const extract = (deviceData, idx) => {
  return deviceData.map((d) => {
    return {t:d.t, y:d.y[idx]};
  });
  }; // extract function
  
  const groupBy = (array, key) => {
  // Return the end result
  return array.reduce((result, currentValue) => {
    // If an array already present for key, push it to the array. Else create an array and push the object
    (result[currentValue[key]] = result[currentValue[key]] || []).push(
      currentValue
    );
    // Return the current iteration `result` value, this will be taken as next iteration `result` value and accumulate
    return result;
  }, {}); // empty object is the initial value for result object
  }; // groupby function    
  
  // IN : data : [{t:epoch_in_ms, y: extracted_data_column}]
  // OUT : [ {t : hours_in_epoch_ms, y : aggregate_for_the_hour }]
  function aggregateDataAvgPerHour(data) {      
  let aggregateData = [];
  let dataMapHours = data.map((d) => {
    return {h:Math.floor(d.t/(3600*1000)), y:d.y}
    });
  
  let dataHourGroups = groupBy(dataMapHours, 'h');
  Object.keys(dataHourGroups).forEach((key) => {
    let arr = dataHourGroups[key];
    let avg = arr.reduce((r,item)=>{return r+item.y;},0) / arr.length;
    aggregateData.push({t:new Date(key*3600*1000), y: avg});
  });
  return aggregateData;
  } // aggregateDataAvgPerHour
  
  // IN : data : [{t:epoch_in_ms, y: extracted_data_column}]
  // OUT : [ {t : hours_in_epoch_ms, y : aggregate_for_the_hour }]
  function aggregateDataMaxPerHour(data) {      
  let aggregateData = [];
  let dataMapHours = data.map((d) => {
    return {h:Math.floor(d.t/(3600*1000)), y:d.y}
    });
  
  let dataHourGroups = groupBy(dataMapHours, 'h');
  Object.keys(dataHourGroups).forEach((key) => {
    let arr = dataHourGroups[key];
    let aMax = arr.reduce((r,item)=>{return Math.max(r,item.y);},0);
    aggregateData.push({t:key*3600*1000, y: aMax});
  });
  return aggregateData;
  } // aggregateDataMaxPerHour
  
  /* MODIFY : this is device specific !! */
  function parseCSV(string) {
  let array = [];
  let lines = string.split("\n");
  for (let i = 1; i < lines.length; i++) { // 1ste lijn is tekst
    let data = lines[i].split(",");
    if (data.length > 1) { // basic dataQ check
      let dt;
      let y = [];
      if (data[0] && (!isNaN(data[0]))) { // timestamp
        dt = 1000* parseInt(data[0]); // datetime, op esp8266 is dat in seconds epoch, geen milliseconds!!
        y.push(parseInt(data[1])); // lastSeenMillis
        for (let i=2;i<=deviceDataItems;i++) { // de andere zijn floats
          if (data[i] != undefined){
            let dataValue = parseFloat(data[i]); // press/hum/temp = float
            y.push(dataValue);
          }
          else {
            y.push(null);
          }
        }
        array.push({t: dt, y : y});
      }
    }
  }
  return array;
  } // parseCSV
  
  function loadCSV() {
  axios.get("tempie.csv")
  .then((res) => {
    deviceData = parseCSV(res.data);
    // init view : filter 1 maand data
    let max_date = Date.now(); // in epoch ms
    let min_date = max_date - 30*24*3600*1000;
    const filterData = (item) => {
      return (item.t > min_date && item.t < max_date);
    };
    let filteredData = deviceData.filter(filterData);
    for (let i=0;i<deviceDataItems;i++) {
      chartData[i] = aggregateDataAvgPerHour(extract(filteredData,i));
      cfg.data.datasets[i].data = chartData[i];
    }
    chart.update();
    }, (err) => {
      console.log(err);
    });
  } // loadCSV

}

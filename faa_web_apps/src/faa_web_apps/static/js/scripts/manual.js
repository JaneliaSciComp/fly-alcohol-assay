// Global functions to be called from actuation_webserver.py
// var update_mfc_flow_rate_settings = function(device,mfc,percent_capacity) {
//     if ((device === 0) || (device === 1)) {
//         $("#odor" + device + "Mfc" + mfc + "Slider").slider( "value", percent_capacity );
//         $("#odor" + device + "Mfc" + mfc + "Display").text(percent_capacity);
//     } else if (device === 2) {
//         $("#ethanolMfc" + mfc + "Slider").slider( "value", percent_capacity );
//         $("#ethanolMfc" + mfc + "Display").text(percent_capacity);
//     }
// };
// var display_device_info = function(device,info) {
//     var tbl = prettyPrint(info, {
//         // Config
//         styles: {
//             'default': {
//                 td: {
//                     fontSize: '1em',
//                     border: '4px solid #000'
//                 },
//                 th: {
//                     fontSize: '1em',
//                     border: '4px solid #000'
//                 }
//             }
//         }
//     });
//     $("#infoDevice"+device).html(tbl);
// };
$(function() {
    var time_display_string = function(time) {
        var hours = Math.floor(time/3600);
        var minutes = Math.floor((time - hours*3600)/60);
        var seconds = Math.floor(time - hours*3600 - minutes*60);
        var display_string = '';
        display_string += (0 < hours ? hours + (1 < hours ? ' hours ' : ' hour  ') : '');
        display_string += (0 < minutes ? minutes + (1 < minutes ? ' minutes ' : ' minute  ') : '');
        display_string += (0 < seconds ? seconds + (1 < seconds ? ' seconds' : ' second ') : '0 seconds');
        return(display_string);
    };

    // Initialize variables
    var tunnelCount = 6;
    var gateCount = 3;
    var olfactometerCount = 3;
    var mfcCount = 2;
    var tableRows = $('#tblStates tr').length;
    var tableCols = tunnelCount;
    $("tr").css("height",100/tableRows + '%');
    $("td").css("width",100/tableCols + '%');
    var secondsElapsed = 0;
    var millisecondsInSecond = 1000;
    var timerAirBefore;
    var timerEthanol;
    var timerAirAfter;
    var timerStart;
    var timerWalk;
    var timerAcclimate;
    var waitTimeEthanolChambersTotal;
    var waitTimeEthanolChambersAirBefore = 0;
    var waitTimeEthanolChambersEthanol = 60;
    var waitTimeEthanolChambersAirAfter = 0;
    var waitTimeStartChambers = 7200;
    var waitingInStartChambers = true;
    var acclimationTime = 120;
    var walkTimes = new Array();
    var walkTimerActivated = new Array();
    var btnWalkEthanolEnabled = false;
    var btnWalkStartEnabled = false;
    // var vialOdor0 = $("#odor0VialSelection :radio:checked").attr('value');
    // var vialOdor1 = $("#odor1VialSelection :radio:checked").attr('value');
    var vialDummy = 0;
    var vialOdor0 = vialDummy;
    var vialOdor1 = vialDummy;
    var vialEthanol = $("#ethanolVialSelection :radio:checked").attr('value');
    var olfactometerOdor0 = 0;
    var olfactometerOdor1 = 1;
    var olfactometerEthanol = 2;
    var timerDisplayFadeTime = 50000;
    var timerRefresh;
    var periodRefresh = 500;
    var imageWidth = 1280;
    var imageHeight = 960;
    var tunnelSliderInitialized = [];
    var gateSliderInitialized = [];


    // jQuery UI
    // tabs
    var tabs = $("#tabs").tabs({
        activate: function( event, ui ) {
            var active = $("#tabs").tabs("option","active");
            if (active !== 1) {
                $("#mjpeg").html('');
            };
            if (active === 1) {
                console.log('tab 1 active');
                clearInterval(timerRefresh);
                timerRefresh = setInterval(refreshImageProcessing,periodRefresh);
                $("#mjpeg").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed" />');
            } else if (active === 2) {
                console.log('tab 2 active');
                saveImageProcessing();
                // $('#btnLoadFlies').button('disable');
                // $('#btnLoadFlies').fadeTo(0,0);
                // saveBackgroundImagesGoal.send();
                $("#tabs").tabs("enable",3);
                clearInterval(timerRefresh);
                timerRefresh = setInterval(refreshFlowRates,periodRefresh);
            } else if (active === 3) {
                console.log('tab 3 active');
                // alert('Save flow rate parameters success');
                $("#tabs").tabs("enable",4);
                clearInterval(timerRefresh);
                timerRefresh = setInterval(refreshExperimentParameters,periodRefresh);
            } else if (active === 4) {
                console.log('not tab 1 or 2 or 3 active');
                saveExperimentParameters();
                parametersInitialized.publish(emptyMsg);
                console.log(timerRefresh);
                clearInterval(timerRefresh);
            };
        },
        heightStyle: "fill"
    });
    $("#tabs").tabs("option","disabled",[3,4]);
    $(window).resize(function() {
        tabs.tabs('refresh');
    });

    // Image Processing
    for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
        tunnelSliderInitialized[tunnel] = false;
    };
    for (var gate=0;gate<gateCount;gate++) {
        gateSliderInitialized[gate] = false;
    };

    // ROS Setup
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });
    // ROS Parameters
    var paramTunnelWidth = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/tunnel_width'
    });
    var paramTunnelHeight = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/tunnel_height'
    });
    var paramTunnelXOffsets = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/tunnel_x_offsets'
    });
    var paramTunnelYOffset = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/tunnel_y_offset'
    });
    var paramGateHeight = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/gate_height'
    });
    var paramGateYOffsets = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/gate_y_offsets'
    });
    var paramImageProcessingLineThickness = new ROSLIB.Param({
      ros : ros,
      name : '/faa_image_processing/calibration_line_thickness'
    });

    var paramExperimentName = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/experiment_name'
    });
    var paramAcclimationDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/acclimation_duration'
    });
    var paramAirBeforeDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/ethanol_chamber/air_before_duration'
    });
    var paramEthanolDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/ethanol_chamber/ethanol_duration'
    });
    var paramEthanolVial = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/ethanol_chamber/ethanol_vial'
    });
    var paramAirAfterDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/ethanol_chamber/air_after_duration'
    });
    // ROS Services
    var saveParamsClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_utilities/save_config_params',
        serviceType : 'faa_utilities/SaveParams'
    });
    var getMfcFlowRateSettingClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/get_mfc_flow_rate_setting',
        serviceType : 'faa_actuation/GetMfcFlowRateSetting'
    });
    var setMfcFlowRateClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/set_mfc_flow_rate',
        serviceType : 'faa_actuation/SetMfcFlowRate'
    });
    var setOdorValveOnClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/set_odor_valve_on',
        serviceType : 'faa_actuation/SetOdorValveOn'
    });
    var setOdorValvesOffClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/set_odor_valves_off',
        serviceType : 'faa_actuation/SetOdorValvesOff'
    });
    var updateGateClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/update_gate',
        serviceType : 'faa_actuation/UpdateGate'
    });
    var updateLedClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_actuation/update_led',
        serviceType : 'faa_actuation/UpdateLed'
    });

    //ROS Actions
    // var saveBackgroundImagesClient = new ROSLIB.ActionClient({
    //     ros : ros,
    //     serverName : '/faa_actionlib/save_background_images',
    //     actionName : 'faa_actionlib/SaveBackgroundImages'
    // });
    // var saveBackgroundImagesGoal = new ROSLIB.Goal({
    //     actionClient : saveBackgroundImagesClient,
    //     goalMessage : {
    //         order: ''
    //     }
    // });
    // saveBackgroundImagesGoal.on('feedback', function(feedback) {
    //     console.log('Feedback: ' + feedback.sequence);
    // });
    // saveBackgroundImagesGoal.on('result', function(result) {
    //     console.log('Final Result: ' + result.status);
    //     $('#btnLoadFlies').button('enable');
    //     $('#btnLoadFlies').fadeTo(0,1);
    // });

    // ROS Topics
  // var cmdVel = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/cmd_vel',
  //   messageType : 'geometry_msgs/Twist'
  // });

  // var twist = new ROSLIB.Message({
  //   linear : {
  //     x : 0.1,
  //     y : 0.2,
  //     z : 0.3
  //   },
  //   angular : {
  //     x : -0.1,
  //     y : -0.2,
  //     z : -0.3
  //   }
  // });
  // cmdVel.publish(twist);
    var parametersInitialized = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_controls/parameters_initialized',
        messageType : 'std_msgs/Empty'
    });
    var emptyMsg = new ROSLIB.Message({
    });

    var updateParamLineThickness = function(event, ui) {
        paramImageProcessingLineThickness.set(ui.value);
        $("#lineThicknessDisplay").text(ui.value);
    };

    $("#lineThicknessSlider").slider({
        range: "min",
        value: 3,
        min: 1,
        max: 5,
        step: 1,
        slide: updateParamLineThickness,
        stop: updateParamLineThickness
    });
    var updateLineThickness = function(param) {
        $("#lineThicknessSlider").slider("option","value",param);
        $("#lineThicknessDisplay").text(param);
    };

    var updateParamTunnelWidth = function(event,ui) {
        paramTunnelWidth.set(ui.value);
        $("#tunnelWidthDisplay").text(ui.value);
    };
    $("#tunnelWidthSlider").slider({
        range: "min",
        value: 100,
        min: 0,
        max: 100,
        step: 1,
        slide: updateParamTunnelWidth,
        stop: updateParamTunnelWidth
    });
    var updateTunnelWidth = function(param) {
        $("#tunnelWidthSlider").slider("option","value",param);
        $("#tunnelWidthDisplay").text(param);
    };

    var updateParamTunnelXOffsets = function(event,ui) {
        var thisTunnel = $(this).attr("data-tunnel");
        $("#tunnelXOffsetDisplay"+thisTunnel).text(ui.value);
        if (!tunnelSliderInitialized[thisTunnel]) {
            tunnelSliderInitialized[thisTunnel] = true;
        };
        var tunnelXOffsets = [];
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            tunnelXOffsets[tunnel] = $('#tunnelXOffsetSlider'+tunnel).slider("option","value");
        }
        // console.log(tunnelXOffsets);
        paramTunnelXOffsets.set(tunnelXOffsets);
    };
    $(".slider.image_processing.xOffset").slider({
        range: "min",
        value: 50,
        min: 0,
        max: 100,
        step: 1,
        slide: updateParamTunnelXOffsets,
        stop: updateParamTunnelXOffsets
    });
    var updateTunnelXOffsets = function(param) {
        var tunnelXOffsets = param;
        // var tunnelCount = tunnelXOffsets.length;
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            var xOffset = tunnelXOffsets[tunnel];
            $("#tunnelXOffsetDisplay"+tunnel).text(xOffset);
            var min = xOffset - 50;
            if (min < 0) {
                min = 0;
            }
            var max = xOffset + 50;
            $('#tunnelXOffsetSlider'+tunnel).slider("option","value",xOffset);
            if (!tunnelSliderInitialized[tunnel]) {
                $('#tunnelXOffsetSlider'+tunnel).slider("option","min",min);
                $('#tunnelXOffsetSlider'+tunnel).slider("option","max",max);
            };
        }
        // console.log(tunnelXOffsets);
    };

    var updateParamTunnelHeight = function(event,ui) {
        paramTunnelHeight.set(ui.value);
        $("#tunnelHeightDisplay").text(ui.value);
    };
    $("#tunnelHeightSlider").slider({
        range: "min",
        value: 900,
        min: 800,
        max: imageHeight,
        step: 1,
        slide: updateParamTunnelHeight,
        stop: updateParamTunnelHeight
    });
    var updateTunnelHeight = function(param) {
        $("#tunnelHeightSlider").slider("option","value",param);
        $("#tunnelHeightDisplay").text(param);
    };

    var updateParamTunnelYOffset = function(event,ui) {
        paramTunnelYOffset.set(ui.value);
        $("#tunnelYOffsetDisplay").text(ui.value);
    };
    $("#tunnelYOffsetSlider").slider({
        range: "min",
        value: 0,
        min: 0,
        max: 100,
        step: 1,
        slide: updateParamTunnelYOffset,
        stop: updateParamTunnelYOffset
    });
    var updateTunnelYOffset = function(param) {
        $("#tunnelYOffsetSlider").slider("option","value",param);
        $("#tunnelYOffsetDisplay").text(param);
    };

    var updateParamGateHeight = function(event,ui) {
        paramGateHeight.set(ui.value);
        $("#gateHeightDisplay").text(ui.value);
    };
    $("#gateHeightSlider").slider({
        range: "min",
        value: 0,
        min: 0,
        max: 100,
        step: 1,
        slide: updateParamGateHeight,
        stop: updateParamGateHeight
    });
    var updateGateHeight = function(param) {
        $("#gateHeightSlider").slider("option","value",param);
        $("#gateHeightDisplay").text(param);
    };

    var updateParamGateYOffsets = function(event,ui) {
        var thisGate = $(this).attr("data-gate");
        $("#gateYOffsetDisplay"+thisGate).text(ui.value);
        if (!gateSliderInitialized[thisGate]) {
            gateSliderInitialized[thisGate] = true;
        };
        var gateYOffsets = [];
        for (var gate=0;gate<gateCount;gate++) {
            gateYOffsets[gate] = $('#gateYOffsetSlider'+gate).slider("option","value");
        }
        // console.log(gateYOffsets);
        paramGateYOffsets.set(gateYOffsets);
    };
    $(".slider.image_processing.yOffset").slider({
        range: "min",
        value: 50,
        min: 0,
        max: 100,
        step: 1,
        slide: updateParamGateYOffsets,
        stop: updateParamGateYOffsets
    });
    var updateGateYOffsets = function(param) {
        var gateYOffsets = param;
        for (var gate=0;gate<gateCount;gate++) {
            var yOffset = gateYOffsets[gate];
            $("#gateYOffsetDisplay"+gate).text(yOffset);
            var min = yOffset - 50;
            if (min < 0) {
                min = 0;
            }
            var max = yOffset + 50;
            $('#gateYOffsetSlider'+gate).slider("option","value",yOffset);
            if (!gateSliderInitialized[gate]) {
                $('#gateYOffsetSlider'+gate).slider("option","min",min);
                $('#gateYOffsetSlider'+gate).slider("option","max",max);
            };
        }
        // console.log(gateYOffsets);
    };


    var saveImageProcessing = function() {
        var saveParamsRequest = new ROSLIB.ServiceRequest({
            params : "faa_image_processing"
        });
        saveParamsClient.callService(saveParamsRequest, function(result) {
            // alert('Save image processing parameters '
            //       + result.status);
            // alert('Result for service call on '
            //       + saveParamsClient.name
            //       + ': '
            //       + result.status);
        });
    };
    // $('#btnSaveImageProcessing').bind('click', function() {
    //     saveImageProcessing();
    // });

    var updateImageSizeDisplay = function(event,ui) {
        var imageWidthNew = Math.round(imageWidth*(ui.value/100));
        var imageHeightNew = Math.round(imageHeight*(ui.value/100));
        $("#mjpeg").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed?width='+imageWidthNew+'?height='+imageHeightNew+'" />');
    };
    $("#imageSizeSlider").slider({
        range: "min",
        value: 100,
        min: 50,
        max: 100,
        step: 25,
        slide: updateImageSizeDisplay,
        stop: updateImageSizeDisplay
    });

    var refreshImageProcessing = function() {
        // updateImageSizeDisplay();
        paramImageProcessingLineThickness.get(updateLineThickness);
        paramTunnelWidth.get(updateTunnelWidth);
        paramTunnelXOffsets.get(updateTunnelXOffsets);
        paramTunnelHeight.get(updateTunnelHeight);
        paramTunnelYOffset.get(updateTunnelYOffset);
        paramGateHeight.get(updateGateHeight);
        paramGateYOffsets.get(updateGateYOffsets);
    };
    var refreshFlowRates = function() {
        for (var device=0;device<olfactometerCount;device++) {
            for (var mfc=0;mfc<mfcCount;mfc++) {
                var request = new ROSLIB.ServiceRequest({
                    device : device,
                    mfc : mfc
                });
                getMfcFlowRateSettingClient.callService(request,
                                                        (function(device,mfc) {
                                                            return function(result) {
                                                                update_mfc_flow_rate_settings(device,mfc,result.percent_capacity);
                                                            };
                                                        })(device,mfc));
                // Sijax.request('updateMfcFlowRateSettings',[device,mfc]);
            };
        };
    };
    var refreshExperimentParameters = function() {
        // paramExperimentName.get(updateExperimentName);
        paramAcclimationDuration.get(updateAcclimationDuration);
        paramAirBeforeDuration.get(updateAirBeforeDuration);
        paramEthanolDuration.get(updateEthanolDuration);
        paramEthanolVial.get(updateEthanolVial);
        paramAirAfterDuration.get(updateAirAfterDuration);
    };
    $("#tabs").tabs("option","active",1);
    // refreshImageProcessing();
    // $('#btnRefreshImageProcessing').bind('click', function() {
    //     refreshImageProcessing();
    // });

    // tab settings
    $('.progressbar').progressbar({
        value: 0
    });
    // hardware info tab
    $(':button').button();
    $('#btnInfo').bind('click', function() {
        // Sijax.request('updateInfo',[]);
    });

    // $("#odor0VialSelectionWalkEthanol").buttonset();
    // $("#odor1VialSelectionWalkEthanol").buttonset();
    // $("#odor0VialSelectionWalkStart").buttonset();
    // $("#odor1VialSelectionWalkStart").buttonset();
    // $(".timepicker").timepicker({
    //     showButtonPanel: false,
    //     showSecond: true
    // });

    // flow rate settings tabs
    // var saveFlowRates = function () {
    //     var saveParamsRequest = new ROSLIB.ServiceRequest({
    //         params : "/faa_actuation/mfc_flow_rates/"
    //     });
    //     saveParamsClient.callService(saveParamsRequest, function(result) {
    //         alert('Result for service call on '
    //               + saveParamsClient.name
    //               + ': '
    //               + result.status);
    //     });
    // };
    // $('#btnSaveFlowRates').bind('click', function() {
    //     saveFlowRates();
    // });

    $(".mfc.slider").slider({
        range: "min",
        value: 100,
        min: 0,
        max: 100,
        step: 1
    });
    $("#odor0Mfc0Slider").on("slide",
                             function( event, ui ) {
                                 $("#odor0Mfc0Display").text(ui.value);
                             });
    $("#odor0Mfc0Slider").on("slidestop",
                             function( event, ui ) {
                                 var request = new ROSLIB.ServiceRequest({
                                     device : 0,
                                     mfc : 0,
                                     percent_capacity: ui.value
                                 });
                                 setMfcFlowRateClient.callService(request, function(result) {
                                 });
                                 // Sijax.request('setMfcFlowRate',[0,0,ui.value]);
                             });
    $("#odor0Mfc1Slider").on("slide",
                             function( event, ui ) {
                                 $("#odor0Mfc1Display").text(ui.value);
                             });
    $("#odor0Mfc1Slider").on("slidestop",
                             function( event, ui ) {
                                 var request = new ROSLIB.ServiceRequest({
                                     device : 0,
                                     mfc : 1,
                                     percent_capacity: ui.value
                                 });
                                 setMfcFlowRateClient.callService(request, function(result) {
                                 });
                                 // Sijax.request('setMfcFlowRate',[0,1,ui.value]);
                             });
    $("#odor1Mfc0Slider").on("slide",
                             function( event, ui ) {
                                 $("#odor1Mfc0Display").text(ui.value);
                             });
    $("#odor1Mfc0Slider").on("slidestop",
                             function( event, ui ) {
                                 var request = new ROSLIB.ServiceRequest({
                                     device : 1,
                                     mfc : 0,
                                     percent_capacity: ui.value
                                 });
                                 setMfcFlowRateClient.callService(request, function(result) {
                                 });
                                 // Sijax.request('setMfcFlowRate',[1,0,ui.value]);
                             });
    $("#odor1Mfc1Slider").on("slide",
                             function( event, ui ) {
                                 $("#odor1Mfc1Display").text(ui.value);
                             });
    $("#odor1Mfc1Slider").on("slidestop",
                             function( event, ui ) {
                                 var request = new ROSLIB.ServiceRequest({
                                     device : 1,
                                     mfc : 1,
                                     percent_capacity: ui.value
                                 });
                                 setMfcFlowRateClient.callService(request, function(result) {
                                 });
                                 // Sijax.request('setMfcFlowRate',[1,1,ui.value]);
                             });
    $("#ethanolMfc0Slider").on("slide",
                               function( event, ui ) {
                                   $("#ethanolMfc0Display").text(ui.value);
                               });
    $("#ethanolMfc0Slider").on("slidestop",
                               function( event, ui ) {
                                   var request = new ROSLIB.ServiceRequest({
                                       device : 2,
                                       mfc : 0,
                                       percent_capacity: ui.value
                                   });
                                   setMfcFlowRateClient.callService(request, function(result) {
                                   });
                                   // Sijax.request('setMfcFlowRate',[2,0,ui.value]);
                               });
    $("#ethanolMfc1Slider").on("slide",
                               function( event, ui ) {
                                   $("#ethanolMfc1Display").text(ui.value);
                               });
    $("#ethanolMfc1Slider").on("slidestop",
                               function( event, ui ) {
                                   var request = new ROSLIB.ServiceRequest({
                                       device : 2,
                                       mfc : 1,
                                       percent_capacity: ui.value
                                   });
                                   setMfcFlowRateClient.callService(request, function(result) {
                                   });
                                   // Sijax.request('setMfcFlowRate',[2,1,ui.value]);
                               });

    // experiment controls tab
    var saveExperimentParameters = function() {
        var saveParamsRequest = new ROSLIB.ServiceRequest({
            params : "faa_experiment"
        });
        saveParamsClient.callService(saveParamsRequest, function(result) {
            // alert('Save experiment parameters '
            //       + result.status);
            // alert('Result for service call on '
            //       + saveParamsClient.name
            //       + ': '
            //       + result.status);
        });
    };
    // $('#btnSaveExperimentParameters').bind('click', function() {
    //     saveExperimentParameters();
    // });
    var updateExperimentName = function(param) {
        $('#experimentName').val(param);
        $('#experimentNameDisplay').text(param);
    };
    $('#btnSetExperimentName').bind('click', function() {
        var experimentName = $('#experimentName').val();
        $('#experimentNameDisplay').text(experimentName);
        paramExperimentName.set(experimentName);
    });
    var updateAcclimationDuration = function(param) {
        $("#acclimationSlider").slider("option","value",param);
        $("#acclimationDisplay").text(time_display_string(param));
        acclimationTime = param;
    };
    var updateParamAcclimationDuration = function(event, ui) {
        $('#acclimationDisplay').text(time_display_string(ui.value));
        paramAcclimationDuration.set(ui.value);
        acclimationTime = ui.value;
    };
    $('#acclimationSlider').slider({
        range: "min",
        value: 120,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAcclimationDuration,
        // function( event, ui ) {
        //     $('#acclimationDisplay').text(time_display_string(ui.value));
        //     acclimationTime = ui.value;
        // }
        stop: updateParamAcclimationDuration
    });
    // ethanol chamber settings
    $("#ethanolVialSelection").buttonset();
    var updateAirBeforeDuration = function(param) {
        $("#ethanolChamberSliderAirBefore").slider("option","value",param);
        $("#ethanolChamberDisplayAirBefore").text(time_display_string(param));
        waitTimeEthanolChambersAirBefore = param;
        update_wait_time_ethanol_chambers();
    };
    var updateParamAirBeforeDuration = function(event, ui) {
        $('#ethanolChamberDisplayAirBefore').text(time_display_string(ui.value));
        waitTimeEthanolChambersAirBefore = ui.value;
        update_wait_time_ethanol_chambers();
        paramAirBeforeDuration.set(ui.value);
    };
    $('#ethanolChamberSliderAirBefore').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAirBeforeDuration,
        // function( event, ui ) {
        //     $('#ethanolChamberDisplayAirBefore').text(time_display_string(ui.value));
        //     waitTimeEthanolChambersAirBefore = ui.value;
        //     update_wait_time_ethanol_chambers();
        // }
        stop: updateParamAirBeforeDuration
    });
    var updateEthanolDuration = function(param) {
        $("#ethanolChamberSliderEthanol").slider("option","value",param);
        $("#ethanolChamberDisplayEthanol").text(time_display_string(param));
        waitTimeEthanolChambersEthanol = param;
        update_wait_time_ethanol_chambers();
    };
    var updateParamEthanolDuration = function(event, ui) {
        $('#ethanolChamberDisplayEthanol').text(time_display_string(ui.value));
        waitTimeEthanolChambersEthanol = ui.value;
        update_wait_time_ethanol_chambers();
        paramEthanolDuration.set(ui.value);
    };
    $('#ethanolChamberSliderEthanol').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamEthanolDuration,
        // function( event, ui ) {
        //     $('#ethanolChamberDisplayEthanol').text(time_display_string(ui.value));
        //     waitTimeEthanolChambersEthanol = ui.value;
        //     update_wait_time_ethanol_chambers();
        // }
        stop: updateParamEthanolDuration
    });
    var updateAirAfterDuration = function(param) {
        $("#ethanolChamberSliderAirAfter").slider("option","value",param);
        $("#ethanolChamberDisplayAirAfter").text(time_display_string(param));
        waitTimeEthanolChambersAirAfter = param;
        update_wait_time_ethanol_chambers();
    };
    var updateParamAirAfterDuration = function(event, ui) {
        $('#ethanolChamberDisplayAirAfter').text(time_display_string(ui.value));
        waitTimeEthanolChambersAirAfter = ui.value;
        update_wait_time_ethanol_chambers();
        paramAirAfterDuration.set(ui.value);
    };
    $('#ethanolChamberSliderAirAfter').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAirAfterDuration,
        // function( event, ui ) {
        //     $('#ethanolChamberDisplayAirAfter').text(time_display_string(ui.value));
        //     waitTimeEthanolChambersAirAfter = ui.value;
        //     update_wait_time_ethanol_chambers();
        // }
        stop: updateParamAirAfterDuration
    });
    var update_wait_time_ethanol_chambers = function() {
        waitTimeEthanolChambersTotal = waitTimeEthanolChambersAirBefore + 
            waitTimeEthanolChambersEthanol + 
            waitTimeEthanolChambersAirAfter;
        $('#ethanolChamberDisplayTotalTime').text(time_display_string(waitTimeEthanolChambersTotal));
    };

    //
    $('.timer.slider').slider({
        range: "min",
        value: 0,
        min: 60,
        max: 7200,
        step: 60,
        slide: function( event, ui ) {
            if (waitingInStartChambers) {
                $('#waitTimeDisplayStart').text(time_display_string(ui.value));
            }
        },
        stop: function( event, ui ) {
            if (waitingInStartChambers) {
                waitTimeStartChambers = ui.value;
            }
        }
    });

    // Settings
    // $("input:radio[name=vialOdor0]").click(function(){
    //     vialOdor0 = $(this).attr("value");
    // });
    // $("input:radio[name=vialOdor1]").click(function(){
    //     vialOdor1 = $(this).attr("value");
    // });
    var updateEthanolVial = function(param) {
        // $("ethanolVial"+param).attr("checked","checked").button('refresh');
        console.log("#ethanolVial"+param);
        $("#ethanolVial"+param).prop("checked", true ).button('refresh');
        // $("input:radio[name=vialEthanol]").attr("checked","checked").button('refresh');
    };
    $("input:radio[name=vialEthanol]").click(function(){
        vialEthanol = parseInt($(this).attr("value"));
        paramEthanolVial.set(vialEthanol);
    });

    var update_mfc_flow_rate_settings = function(device,mfc,percent_capacity) {
        // console.log("device: " + device + " mfc: " + mfc + " percent_capacity: " + percent_capacity);
        if ((device === 0) || (device === 1)) {
            $("#odor" + device + "Mfc" + mfc + "Slider").slider( "value", percent_capacity );
            $("#odor" + device + "Mfc" + mfc + "Display").text(percent_capacity);
        } else if (device === 2) {
            $("#ethanolMfc" + mfc + "Slider").slider( "value", percent_capacity );
            $("#ethanolMfc" + mfc + "Display").text(percent_capacity);
        }
    };
    var initialize_tabs = function () {
        update_wait_time_ethanol_chambers();
        $('#ethanolChamberDisplayAirBefore').text(time_display_string(waitTimeEthanolChambersAirBefore));
        $('#ethanolChamberSliderAirBefore').slider('value',waitTimeEthanolChambersAirBefore);
        $('#ethanolChamberDisplayEthanol').text(time_display_string(waitTimeEthanolChambersEthanol));
        $('#ethanolChamberSliderEthanol').slider('value',waitTimeEthanolChambersEthanol);
        $('#ethanolChamberDisplayAirAfter').text(time_display_string(waitTimeEthanolChambersAirAfter));
        $('#ethanolChamberSliderAirAfter').slider('value',waitTimeEthanolChambersAirAfter);
        $('#ethanolChamberDisplayTotalTime').text(time_display_string(waitTimeEthanolChambersTotal));
        $('#acclimationDisplay').text(time_display_string(acclimationTime));
    };
    initialize_tabs();

    // General functions
    var reset_walk_times = function() {
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            walkTimes[tunnel] = 0;
            walkTimerActivated[tunnel] = true;
        }
        display_walk_times();
    };
    var display_walk_times = function() {
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#elapsedTimeDisplayWalk'+tunnel).text(time_display_string(walkTimes[tunnel]));
        }
    };
    var timer_walk = function() {
        secondsElapsed += 1;
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            if (walkTimerActivated[tunnel]) {
                walkTimes[tunnel] = secondsElapsed;
            }
        }
        display_walk_times();
    };
    var end_acclimation = function() {
        $('.timer.acclimate.progressbar').progressbar('disable');
        $('.timer.acclimate.display').attr('disabled',true);
        $('#btnEndAcclimation').button('disable');
        $('.acclimate').fadeTo('slow',0,
                               function() {
                                   $('#elapsedTimeDisplayAcclimation').text(time_display_string(0));
                                   $('#elapsedTimeProgressbarAcclimation').progressbar( "option", "value", 0 );
                               });

        clearInterval(timerAcclimate);
        walk_to_start_chambers(false);
    };
    var timer_acclimate = function() {
        secondsElapsed += 1;
        $('#elapsedTimeDisplayAcclimation').text(time_display_string(secondsElapsed));
        $('#elapsedTimeProgressbarAcclimation').progressbar( "option", "value", (secondsElapsed/acclimationTime)*100);
        if (acclimationTime <= secondsElapsed) {
            end_acclimation();
        };
    };

    // Initialize button and timer states
    var initialize = function () {
        $('#btnReset').button('enable');
        $('#btnReset').fadeTo(0,1);
        $('#btnLoadFlies').button('enable');
        $('#btnLoadFlies').fadeTo(0,1);
        $('#btnAcclimateFlies').button('disable');
        $('#btnEndAcclimation').button('disable');
        $('.timer.acclimate.progressbar').progressbar('disable');
        $('.timer.acclimate.display').attr('disabled',true);
        $('.acclimate').fadeTo(0,0);
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#btnRemoveFlies'+tunnel).text('Remove Fly ' + (tunnel + 1));
            $('#btnReopenEthanol'+tunnel).text('Reopen Ethanol Chamber ' + (tunnel + 1));
            $('#btnReopenStart'+tunnel).text('Reopen Start Chamber ' + (tunnel + 1));
        }
        $('.button.ethanol').button('disable');
        $('.timer.ethanol.slider').slider('disable');
        $('.timer.ethanol.progressbar').progressbar('disable');
        $('.timer.ethanol.display').attr('disabled',true);
        $('.ethanol').fadeTo(0,0);
        $('.button.start').button('disable');
        $('.timer.start.slider').slider('disable');
        $('.timer.start.progressbar').progressbar('disable');
        $('.timer.start.display').attr('disabled',true);
        $('.start').fadeTo(0,0);
        reset_walk_times();
        $('.timer.walk.display').attr('disabled',true);
        $('.timer.walk.display').fadeTo(0,0);
        $('.progressbar').progressbar( "option", "value", 0);
        clearInterval(timerWalk);
        clearInterval(timerAcclimate);
    };
    initialize();
    var reset_wait_ethanol = function() {
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#btnWaitEthanol'+tunnel).text('Wait in Ethanol Chamber ' + (tunnel + 1));
        }
        clearInterval(timerAirBefore);
        clearInterval(timerEthanol);
        clearInterval(timerAirAfter);
        $('.button.ethanol').button('disable');
        $('.button.ethanol').fadeTo('slow',0);
        // $('.btnWait.ethanol').button('disable');
        // $('.btnWait.ethanol').fadeTo('slow',0);
        // $('.btnReopen.ethanol').button('disable');
        // $('.btnReopen.ethanol').fadeTo('slow',0);
        $('.timer.ethanol.slider').slider('disable');
        $('.timer.ethanol.progressbar').progressbar('disable');
        $('.timer.ethanol.display').attr('disabled',true);
        $('.timer.ethanol').fadeTo('slow',0,
                                   function() {
                                       $('#elapsedTimeDisplayAirBefore').text("Elapsed Time Air Before: (" + time_display_string(0) + ")/(" + time_display_string(waitTimeEthanolChambersAirBefore) + ")");
                                       $('#elapsedTimeDisplayEthanol').text("Elapsed Time Ethanol: (" + time_display_string(0) + ")/(" + time_display_string(waitTimeEthanolChambersEthanol) + ")");
                                       $('#elapsedTimeDisplayAirAfter').text("Elapsed Time Air After: (" + time_display_string(0) + ")/(" + time_display_string(waitTimeEthanolChambersAirAfter) + ")");
                                       $('#elapsedTimeDisplayEthanolTotal').text("Elapsed Time Ethanol Chambers Total: (" + time_display_string(0) + ")/(" + time_display_string(waitTimeEthanolChambersTotal) + ")");
                                       $('.timer.ethanol.progressbar').progressbar( "option", "value", 0 );
                                   });
    };
    reset_wait_ethanol();
    var reset_wait_start = function() {
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#btnWaitStart'+tunnel).text('Wait in Start Chamber ' + (tunnel + 1));
        }
        clearInterval(timerStart);
        $('.button.start').button('disable');
        $('.button.start').fadeTo('slow',0);
        // $('.btnWait.start').button('disable');
        // $('.btnWait.start').fadeTo('slow',0);
        // $('.btnReopen.start').button('disable');
        // $('.btnReopen.start').fadeTo('slow',0);
        $('.timer.start.slider').slider('disable');
        $('.timer.start.progressbar').progressbar('disable');
        $('.timer.start.display').attr('disabled',true);
        $('.timer.start').fadeTo('slow',0,
                                 function() {
                                     $('#elapsedTimeDisplayStart').text(time_display_string(0));
                                     $('#elapsedTimeProgressbarStart').progressbar( "option", "value", 0 );
                                 });
    };
    reset_wait_start();

    // Experiment Step A: Load Flies
    $('#btnLoadFlies').bind('click', function() {
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        var request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : 'all',
            angle : 'close'
        });
        updateGateClient.callService(request, function(result) {
            // alert('Update gate '
            //       + result.status);
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            led : 'all',
            duty_cycle : 'off'
        });
        updateLedClient.callService(request, function(result) {
        });
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor0]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor1]);
        // // Sijax.request('setOdorValvesOff',[olfactometerEthanol]);
        // Sijax.request('updateGate',['all','all','close']);
        // Sijax.request('updateLed',['all','all','off']);
        $('#btnAcclimateFlies').button('enable');
        $('#btnAcclimateFlies').fadeTo('slow',1);
    });

    // Experiment Step B: Acclimate Flies
    $('#btnAcclimateFlies').bind('click', function() {
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        var request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '0',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
            // alert('Update gate '
            //       + result.status);
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            led : 'all',
            duty_cycle : 'off'
        });
        updateLedClient.callService(request, function(result) {
        });
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // Sijax.request('updateGate',['all',0,'open']);
        // Sijax.request('updateLed',['all','all','off']);
        $('.timer.acclimate.display').attr('disabled',false);
        $('.timer.acclimate.progressbar').progressbar('enable');
        $('.timer.acclimate').fadeTo('slow',1);
        $('#btnEndAcclimation').button('enable');
        $('#btnEndAcclimation').fadeTo('slow',1);
        secondsElapsed = 0;
        timerAcclimate = setInterval(timer_acclimate,millisecondsInSecond);
    });
    $('#btnEndAcclimation').bind('click', function() {
        end_acclimation();
    });

    // Experiment Step C: Run Trials

    // Trial Step 1: Walk to Ethanol Chambers
    $('#btnWalkEthanol').bind('click', function() {
        walk_to_ethanol_chambers();
    });
    var walk_to_ethanol_chambers = function() {
        $('#btnWalkEthanol').button('disable');
        $('#btnWalkEthanol').fadeTo('slow',0);

        // Clear conditions from wait in start chambers
        reset_wait_start();

        reset_walk_times();
        secondsElapsed = 0;
        timerWalk = setInterval(timer_walk,millisecondsInSecond);

        var request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialOdor0
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialOdor1
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '0',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '1',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialOdor0]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialOdor1]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerEthanol]);
        // Sijax.request('updateGate',['all',0,'open']);
        // Sijax.request('updateGate',['all',1,'open']);
        $('.btnWait.ethanol').button('enable');
        $('.btnWait.ethanol').fadeTo('slow',1);
        $('.timer.walk.display').attr('disabled',false);
        $('.timer.walk.display').fadeTo('slow',1);
    };

    // Trial Step 2: Wait in Ethanol Chambers
    $('.btnWait.ethanol').bind('click', function() {
        var self = this;
        var tunnel = Number($(self).val());
        walkTimerActivated[tunnel] = false;
        var request = new ROSLIB.ServiceRequest({
            tunnel : new String(tunnel),
            gate : '1',
            angle : 'close'
        });
        updateGateClient.callService(request, function(result) {
        });
        // Sijax.request('updateGate',[tunnel,1,'close']);
        $(self).button('disable');
        $(self).fadeTo('fast',0,
                       function() {
                           $(self).text('Wait in Ethanol Chamber 0');
                           $('#btnReopenEthanol'+tunnel).button('enable');
                           $('#btnReopenEthanol'+tunnel).fadeTo('fast',1);
                           check_ethanol_chambers_ready(tunnel);
                       });
    });
    var check_ethanol_chambers_ready = function(tunnel) {
        var allTunnelsReady = true;
        for (var t=0;t<tunnelCount;t++) {
            allTunnelsReady = allTunnelsReady && ($('#btnWaitEthanol'+t).text() == 'Wait in Ethanol Chamber 0');
        };
        if (allTunnelsReady) {
            $('#btnEthanolReady').button('enable');
            $('#btnEthanolReady').fadeTo('fast',1);
        };
    };
    $('#btnEthanolReady').bind('click', function() {
        var self = this;
        var tunnel = Number($(self).val());
        $(self).button('disable');
        $('.btnReopen.ethanol').button('disable');
        $('.btnReopen.ethanol').fadeTo('fast',0);
        $(self).fadeTo('fast',0,
                       function() {
                           wait_in_ethanol_chambers(tunnel);
                       });
    });
    var wait_in_ethanol_chambers = function(tunnel) {
        clearInterval(timerWalk);
        waitingInStartChambers = false;
        var request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor0]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor1]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        secondsElapsed = 0;
        timerAirBefore = setInterval(timer_air_before,millisecondsInSecond);
        // $('#waitTimeSliderEthanol').slider('value',waitTimeEthanolChambersTotal);
        // $('#waitTimeDisplayEthanol').text(time_display_string(waitTimeEthanolChambersTotal));
        
        $('.timer.ethanol.display').attr('disabled',false);
        // $('.timer.ethanol.slider').slider('enable');
        $('.timer.ethanol.progressbar').progressbar('enable');
        $('.timer.ethanol').fadeTo('slow',1);
        $('.timer.walk.display').attr('disabled',true);
        btnWalkStartEnabled = false;
        enable_btnWalkStart();
        // $('.timer.walk.display').fadeTo(timerDisplayFadeTime,0,function(){
        //     // triggers on all matched elements, enable only once
        //     enable_btnWalkStart();
        // });
    };
    var enable_btnWalkStart = function () {
        if (!btnWalkStartEnabled) {
            $('#btnWalkStart').button('enable');
            $('#btnWalkStart').fadeTo('slow',1);
            btnWalkStartEnabled = true;
        };
    };
    var timer_air_before = function() {
        secondsElapsed += 1;
        $('#elapsedTimeDisplayAirBefore').text("Elapsed Time Air Before: (" + time_display_string(secondsElapsed) + ")/(" + time_display_string(waitTimeEthanolChambersAirBefore) + ")");
        $('#elapsedTimeProgressbarAirBefore').progressbar( "option", "value", (secondsElapsed/waitTimeEthanolChambersAirBefore)*100);
        $('#elapsedTimeDisplayEthanolTotal').text("Elapsed Time Ethanol Chambers Total: (" + time_display_string(secondsElapsed) + ")/(" + time_display_string(waitTimeEthanolChambersTotal) + ")");
        $('#elapsedTimeProgressbarEthanolTotal').progressbar( "option", "value", (secondsElapsed/waitTimeEthanolChambersTotal)*100);
        if (waitTimeEthanolChambersAirBefore <= secondsElapsed) {
            clearInterval(timerAirBefore);
            var request = new ROSLIB.ServiceRequest({
                device : olfactometerEthanol,
                valve : vialEthanol
            });
            setOdorValveOnClient.callService(request, function(result) {
            });
            // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialEthanol]);
            secondsElapsed = 0;
            timerEthanol = setInterval(timer_ethanol,millisecondsInSecond);
        };
    };
    var timer_ethanol = function() {
        secondsElapsed += 1;
        $('#elapsedTimeDisplayEthanol').text("Elapsed Time Ethanol: (" + time_display_string(secondsElapsed) + ")/(" + time_display_string(waitTimeEthanolChambersEthanol) + ")");
        $('#elapsedTimeProgressbarEthanol').progressbar( "option", "value", (secondsElapsed/waitTimeEthanolChambersEthanol)*100);
        $('#elapsedTimeDisplayEthanolTotal').text("Elapsed Time Ethanol Chambers Total: (" + time_display_string(secondsElapsed+waitTimeEthanolChambersAirBefore) + ")/(" + time_display_string(waitTimeEthanolChambersTotal) + ")");
        $('#elapsedTimeProgressbarEthanolTotal').progressbar( "option", "value", ((secondsElapsed+waitTimeEthanolChambersAirBefore)/waitTimeEthanolChambersTotal)*100);
        if (waitTimeEthanolChambersEthanol <= secondsElapsed) {
            clearInterval(timerEthanol);
            var request = new ROSLIB.ServiceRequest({
                device : olfactometerEthanol,
                valve : vialDummy
            });
            setOdorValveOnClient.callService(request, function(result) {
            });
            // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
            secondsElapsed = 0;
            timerAirAfter = setInterval(timer_air_after,millisecondsInSecond);
        };
    };
    var timer_air_after = function() {
        secondsElapsed += 1;
        $('#elapsedTimeDisplayAirAfter').text("Elapsed Time Air After: (" + time_display_string(secondsElapsed) + ")/(" + time_display_string(waitTimeEthanolChambersAirAfter) + ")");
        $('#elapsedTimeProgressbarAirAfter').progressbar( "option", "value", (secondsElapsed/waitTimeEthanolChambersAirAfter)*100);
        $('#elapsedTimeDisplayEthanolTotal').text("Elapsed Time Ethanol Chambers Total: (" + time_display_string(secondsElapsed+waitTimeEthanolChambersAirBefore+waitTimeEthanolChambersEthanol) + ")/(" + time_display_string(waitTimeEthanolChambersTotal) + ")");
        $('#elapsedTimeProgressbarEthanolTotal').progressbar( "option", "value", ((secondsElapsed+waitTimeEthanolChambersAirBefore+waitTimeEthanolChambersEthanol)/waitTimeEthanolChambersTotal)*100);
        if (waitTimeEthanolChambersAirAfter <= secondsElapsed) {
            clearInterval(timerAirAfter);
            secondsElapsed = 0;
            walk_to_start_chambers(true);
        };
    };
    $('.btnReopen.ethanol').bind('click', function() {
        var self = this;
        var tunnel = Number($(self).val());
        walkTimerActivated[tunnel] = true;
        request = new ROSLIB.ServiceRequest({
            tunnel : new String(tunnel),
            gate : '1',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        // Sijax.request('updateGate',[tunnel,1,'open']);
        $(self).button('disable');
        $(self).fadeTo('fast',0,
                       function() {
                           $('#btnWaitEthanol'+tunnel).text('Wait in Ethanol Chamber ' + (tunnel + 1));
                           $('#btnWaitEthanol'+tunnel).button('enable');
                           $('#btnWaitEthanol'+tunnel).fadeTo('fast',1);
                           $('#btnEthanolReady').button('disable');
                           $('#btnEthanolReady').fadeTo('fast',0);
                       });
    });

    // Trial Step 3: Walk to Start Chambers
    $('#btnWalkStart').bind('click', function() {
        walk_to_start_chambers(true);
    });
    var walk_to_start_chambers = function(openGate1) {
        $('#btnWalkStart').button('disable');
        $('#btnWalkStart').fadeTo('slow',0);

        // Clear conditions from wait in ethanol chambers
        reset_wait_ethanol();

        reset_walk_times();
        secondsElapsed = 0;
        timerWalk = setInterval(timer_walk,millisecondsInSecond);

        var request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            led : '0',
            duty_cycle : 'on'
        });
        updateLedClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        // Sijax.request('updateLed',['all',0,'on']);
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor0]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor1]);
        // // Sijax.request('setOdorValvesOff',[olfactometerEthanol]);
        if (openGate1) {
            request = new ROSLIB.ServiceRequest({
                tunnel : 'all',
                gate : '1',
                angle : 'open'
            });
            updateGateClient.callService(request, function(result) {
            });
            // Sijax.request('updateGate',['all',1,'open']);
        }
        $('.btnWait.start').button('enable');
        $('.btnWait.start').fadeTo('slow',1);
        $('.timer.walk.display').attr('disabled',false);
        $('.timer.walk.display').fadeTo('slow',1);
    };

    // Trial Step 4: Wait in Start Chambers
    $('.btnWait.start').bind('click', function() {
        var self = this;
        var tunnel = $(self).val();
        walkTimerActivated[tunnel] = false;
        var request = new ROSLIB.ServiceRequest({
            tunnel : new String(tunnel),
            gate : '0',
            angle : 'close'
        });
        updateGateClient.callService(request, function(result) {
        });
        // Sijax.request('updateGate',[tunnel,0,'close']);
        $(self).button('disable');
        $(self).fadeTo('fast',0,
                       function() {
                           $(self).text('Wait in Start Chamber 0');
                           $('#btnReopenStart'+tunnel).button('enable');
                           $('#btnReopenStart'+tunnel).fadeTo('fast',1);
                           check_start_chambers_ready(tunnel);
                       });
    });
    var check_start_chambers_ready = function(tunnel) {
        var allTunnelsReady = true;
        for (var t=0;t<tunnelCount;t++) {
            allTunnelsReady = allTunnelsReady && ($('#btnWaitStart'+t).text() == 'Wait in Start Chamber 0');
        };
        if (allTunnelsReady) {
            $('#btnStartReady').button('enable');
            $('#btnStartReady').fadeTo('fast',1);
        };
    };
    $('#btnStartReady').bind('click', function() {
        var self = this;
        var tunnel = Number($(self).val());
        $(self).button('disable');
        $('.btnReopen.start').button('disable');
        $('.btnReopen.start').fadeTo('fast',0);
        $(self).fadeTo('fast',0,
                       function() {
                           wait_in_start_chambers(tunnel);
                       });
    });
    var wait_in_start_chambers = function(tunnel) {
        clearInterval(timerWalk);
        waitingInStartChambers = true;
        var request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            led : '0',
            duty_cycle : 'off'
        });
        updateLedClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        // Sijax.request('updateLed',['all',0,'off']);
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor0]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor1]);
        // // Sijax.request('setOdorValvesOff',[olfactometerEthanol]);
        secondsElapsed = 0;
        timerStart = setInterval(timer_start,millisecondsInSecond);
        $('#waitTimeSliderStart').slider('value',waitTimeStartChambers);
        $('#waitTimeDisplayStart').text(time_display_string(waitTimeStartChambers));
        $('.timer.start.display').attr('disabled',false);
        $('.timer.start.slider').slider('enable');
        $('.timer.start.progressbar').progressbar('enable');
        $('.timer.start').fadeTo('slow',1);
        $('.timer.walk.display').attr('disabled',true);
        btnWalkEthanolEnabled = false;
        enable_btnWalkEthanol();
        // $('.timer.walk.display').fadeTo(timerDisplayFadeTime,0,function(){
        //     // triggers on all matched elements, enable only once
        //     // enable_btnWalkEthanol();
        // });
    };
    var enable_btnWalkEthanol = function () {
        if (!btnWalkEthanolEnabled) {
            $('#btnWalkEthanol').button('enable');
            $('#btnWalkEthanol').fadeTo('slow',1);
            btnWalkEthanolEnabled = true;
        };
    };
    var timer_start = function() {
        secondsElapsed += 1;
        $('#elapsedTimeDisplayStart').text(time_display_string(secondsElapsed));
        $('#elapsedTimeProgressbarStart').progressbar( "option", "value", (secondsElapsed/waitTimeStartChambers)*100);
        if (waitTimeStartChambers <= secondsElapsed) {
            walk_to_ethanol_chambers();
        };
    };
    $('.btnReopen.start').bind('click', function() {
        var self = this;
        var tunnel = Number($(self).val());
        walkTimerActivated[tunnel] = true;
        var request = new ROSLIB.ServiceRequest({
            tunnel : new String(tunnel),
            gate : '0',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        // Sijax.request('updateGate',[tunnel,0,'open']);
        $(self).button('disable');
        $(self).fadeTo('fast',0,
                       function() {
                           $('#btnWaitStart'+tunnel).text('Wait in Start Chamber ' + (tunnel + 1));
                           $('#btnWaitStart'+tunnel).button('enable');
                           $('#btnWaitStart'+tunnel).fadeTo('fast',1);
                           $('#btnStartReady').button('disable');
                           $('#btnStartReady').fadeTo('fast',0);
                       });
    });

    // Optional Step: Remove Flies
    //$('.btnRemoveFlies').bind('click', function() {
    //});

    // Start a new trial, jump to trial step 1

    // Reset
    $('#btnReset').bind('click', function() {
        $('.button').stop(true,false);
        $('.timer').stop(true,false);
        initialize();
        reset_wait_ethanol();
        reset_wait_start();
        var request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor0,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerOdor1,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            device : olfactometerEthanol,
            valve : vialDummy
        });
        setOdorValveOnClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '0',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '1',
            angle : 'open'
        });
        updateGateClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            gate : '2',
            angle : 'close'
        });
        updateGateClient.callService(request, function(result) {
        });
        request = new ROSLIB.ServiceRequest({
            tunnel : 'all',
            led : '0',
            duty_cycle : 'on'
        });
        updateLedClient.callService(request, function(result) {
        });
        // Sijax.request('setOdorValveOn',[olfactometerOdor0,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerOdor1,vialDummy]);
        // Sijax.request('setOdorValveOn',[olfactometerEthanol,vialDummy]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor0]);
        // // Sijax.request('setOdorValvesOff',[olfactometerOdor1]);
        // // Sijax.request('setOdorValvesOff',[olfactometerEthanol]);
        // // Sijax.request('updateGate',['all','all','open']);
        // Sijax.request('updateGate',['all',0,'open']);
        // Sijax.request('updateGate',['all',1,'open']);
        // Sijax.request('updateGate',['all',2,'close']);
        // // Sijax.request('updateLed',['all','all','off']);
        // Sijax.request('updateLed',['all',0,'on']);
    });

});

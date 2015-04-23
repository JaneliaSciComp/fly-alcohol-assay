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

    // Tab names
    var tabHardwareInfo = 0;
    var tabSaveBackgroundImages = 1;
    var tabImageProcessingSettings = 2;
    var tabFlowRateSettings = 3;
    var tabExperimentSettings = 4;
    var tabExperimentControls = 5;

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

    var waitTimeEndChambersTotal;
    var waitTimeEndChambersAirBefore = 0;
    var waitTimeEndChambersEthanol = 60;
    var waitTimeEndChambersAirAfter = 0;
    var waitTimeStartChambers = 7200;
    var acclimationDuration = 120;
    var trialCount = 5;
    var startChamberDuration = 120;
    var timeoutDuration = 240;

    var vialEthanol = $("#ethanolVialSelection :radio:checked").attr('value');

    var timerRefresh;
    var periodRefresh = 500;
    var tunnelSliderInitialized = [];
    var gateSliderInitialized = [];

    var imageWidth = 1280;
    var imageHeight = 960;
    var image_processing_parameters;

    var tunnelsEnabled = [];

    var timerDuration;
    var periodDuration = 1000;
    var statusDurationTotal;
    var statusDurationRemaining;

    var tunnel_y_offset;

    // jQuery UI
    // tabs
    var tabs = $("#tabs").tabs({
        activate: function( event, ui ) {
            var active = $("#tabs").tabs("option","active");
            if (active !== tabHardwareInfo) {
                $("#mjpegSettings").html('');
            };
            if (active !== tabSaveBackgroundImages) {
                $("#mjpegSaveBackgroundImages").html('');
            };
            if (active !== tabExperimentControls) {
                $("#mjpegExperiment").html('');
            };
            if (active === tabSaveBackgroundImages) {
                console.log('tab Save Background Images active');
                timerRefresh = setInterval(refreshSaveBackgroundImages,periodRefresh);
                $("#mjpegSaveBackgroundImages").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed" />');
            } else if (active === tabImageProcessingSettings) {
                console.log('tab Image Processing Settings active');
                $('#tabs').tabs('disable',tabSaveBackgroundImages);
                $("#tabs").tabs("enable",tabFlowRateSettings);
                clearInterval(timerRefresh);
                // timerRefresh = setInterval(refreshImageProcessing,periodRefresh);
                getImageProcessingParams();
                $("#mjpegSettings").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed" />');
            } else if (active === tabFlowRateSettings) {
                console.log('tab Flow Rate Settings active');
                saveImageProcessing();
                $("#tabs").tabs("enable",tabExperimentSettings);
                clearInterval(timerRefresh);
                timerRefresh = setInterval(refreshFlowRates,periodRefresh);
            } else if (active === tabExperimentSettings) {
                console.log('tab Experiment Settings active');
                $("#tabs").tabs("enable",tabExperimentControls);
                clearInterval(timerRefresh);
                timerRefresh = setInterval(refreshExperimentParameters,periodRefresh);
            } else if (active === tabExperimentControls) {
                console.log('tab Experiment Controls active');
                $("#tabs").tabs("option","disabled",[tabSaveBackgroundImages,tabImageProcessingSettings,tabFlowRateSettings,tabExperimentSettings]);
                saveExperimentParameters();
                parametersInitialized.publish(emptyMsg);
                $("#mjpegExperiment").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed" />');
                console.log(timerRefresh);
                clearInterval(timerRefresh);
                $('#status').text("Status: Setup");
                $('#statusDuration').text("");
                $('#statusTunnelsToCatch').text("");
                $('#trialCountdown').text("");
            };
        },
        heightStyle: "fill"
    });
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
    var paramExperimentName = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/experiment_name'
    });
    var paramTrialCount = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/trial_count'
    });
    var paramAcclimationDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/acclimation_duration'
    });
    var paramStartChamberDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/start_chamber_duration'
    });
    var paramAirBeforeDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/air_before_duration'
    });
    var paramEthanolDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/ethanol_duration'
    });
    var paramEthanolVial = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/ethanol_vial'
    });
    var paramAirAfterDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/air_after_duration'
    });
    var paramLightsEnabled = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/lights_enabled'
    });
    var paramLightsPercentCapacity = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/lights_percent_capacity'
    });
    var paramLightsDurationOn = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/lights_duration_on'
    });
    var paramLightsDurationOff = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/end_chamber/lights_duration_off'
    });
    var paramTunnelsEnabled = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/tunnels_enabled'
    });
    var paramTimeoutDuration = new ROSLIB.Param({
      ros : ros,
      name : '/faa_experiment/timeout_duration'
    });
    var paramActuationInitialized = new ROSLIB.Param({
      ros : ros,
      name : '/faa_actuation/initialized'
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
    var getImageProcessingParamsClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_image_processing/get_parameters',
        serviceType : 'faa_image_processing/GetParameters'
    });
    var setImageProcessingParamClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_image_processing/set_parameter',
        serviceType : 'faa_image_processing/SetParameter'
    });
    var setImageProcessingArrayParamClient = new ROSLIB.Service({
        ros : ros,
        name : '/faa_image_processing/set_array_parameter',
        serviceType : 'faa_image_processing/SetArrayParameter'
    });

    // ROS Actions

    // ROS Topics
    var saveBackgroundImages = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_controls/save_background_images',
        messageType : 'std_msgs/Empty'
    });
    var parametersInitialized = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_controls/parameters_initialized',
        messageType : 'std_msgs/Empty'
    });
    var loadFlies = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_controls/load_flies',
        messageType : 'std_msgs/Empty'
    });
    var runExperiment = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_controls/run_experiment',
        messageType : 'std_msgs/Empty'
    });
    var emptyMsg = new ROSLIB.Message({
    });
    var experimentStatus = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_status/status',
        messageType : 'faa_status/Status'
    });
    var experimentDuration = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_status/duration',
        messageType : 'faa_status/Duration'
    });
    var experimentTunnelsToCatch = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_status/tunnels_to_catch',
        messageType : 'faa_status/TunnelsToCatch'
    });
    var experimentTunnelsEnabled = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_status/tunnels_enabled',
        messageType : 'faa_status/TunnelsEnabled'
    });
    var experimentTrial = new ROSLIB.Topic({
        ros : ros,
        name : '/faa_status/trial',
        messageType : 'faa_status/Trial'
    });
    experimentStatus.subscribe(function(message) {
        $('#status').text("Status: " + message.status);
    });
    experimentTrial.subscribe(function(message) {
        $('#trialCountdown').text("Trial: " + message.trial + "/" + message.trial_count);
    });
    var durationUpdate = function() {
        if (statusDurationRemaining > 0) {
            var statusWithCount = 'Duration: ' + statusDurationRemaining + '/' + statusDurationTotal;
            $('#statusDuration').text(statusWithCount);
            statusDurationRemaining -= 1;
        } else {
            clearInterval(timerDuration);
            $('#statusDuration').text("");
        };
    };
    experimentDuration.subscribe(function(message) {
        clearInterval(timerDuration);
        $('#statusDuration').text("");
        if (message.duration > 0 ) {
            statusDurationTotal = message.duration;
            statusDurationRemaining = message.duration - 1;
            timerDuration = setInterval(durationUpdate,periodDuration);
        };
    });
    experimentTunnelsToCatch.subscribe(function(message) {
        if (message.tunnels_to_catch.length > 0) {
            var ttc = [];
            for (var i=0; i<message.tunnels_to_catch.length; i++){
                ttc[i] = Number(message.tunnels_to_catch[i]) + 1;
            }
            $('#statusTunnelsToCatch').text('Tunnels To Catch: ' + ttc);
        } else {
            $('#statusTunnelsToCatch').text('');
        };
    });
    experimentTunnelsEnabled.subscribe(function(message) {
        updateTunnelsEnabled(message.tunnels_enabled);
    });

    // Get Image Processing Parameters
    var updateLineThickness = function(param) {
        $("#lineThicknessSlider").slider("option","value",param);
        $("#lineThicknessDisplay").text(param);
    };
    var updateGateHeight = function(param) {
        $("#gateHeightSlider").slider("option","value",param);
        $("#gateHeightDisplay").text(param);
    };
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
    };
    var updateTunnelHeight = function(param) {
        var tunnelHeight = param;
        $("#tunnelHeightSlider").slider("option","max",(imageHeight-tunnel_y_offset));
        if ((tunnel_y_offset + tunnelHeight) > imageHeight) {
            tunnelHeight = imageHeight - tunnel_y_offset;
        };
        $("#tunnelHeightSlider").slider("option","value",tunnelHeight);
        $("#tunnelHeightDisplay").text(tunnelHeight);
    };
    var updateTunnelWidth = function(param) {
        $("#tunnelWidthSlider").slider("option","value",param);
        $("#tunnelWidthDisplay").text(param);
    };
    var updateTunnelXOffsets = function(param) {
        var tunnelXOffsets = param;
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
    };
    var updateTunnelYOffset = function(param) {
        $("#tunnelYOffsetSlider").slider("option","value",param);
        $("#tunnelYOffsetDisplay").text(param);
        tunnel_y_offset = param;
    };
    var getImageProcessingParams = function() {
        var getImageProcessingParamsRequest = new ROSLIB.ServiceRequest({
        });
        getImageProcessingParamsClient.callService(getImageProcessingParamsRequest, function(result) {
            var parameters = $.parseJSON(result.parameters);
            updateLineThickness(parameters.calibration_line_thickness);
            updateGateHeight(parameters.gate_height);
            updateGateYOffsets(parameters.gate_y_offsets);
            updateTunnelYOffset(parameters.tunnel_y_offset);
            updateTunnelHeight(parameters.tunnel_height);
            updateTunnelWidth(parameters.tunnel_width);
            updateTunnelXOffsets(parameters.tunnel_x_offsets);
        });
    };

    var updateParamLineThickness = function(event, ui) {
        var request = new ROSLIB.ServiceRequest({
            name : 'calibration_line_thickness',
            value: ui.value
        });
        setImageProcessingParamClient.callService(request, function(result) {
        });
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

    var updateParamGateHeight = function(event,ui) {
        var request = new ROSLIB.ServiceRequest({
            name : 'gate_height',
            value: ui.value
        });
        setImageProcessingParamClient.callService(request, function(result) {
        });
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

    var updateParamTunnelHeight = function(event,ui) {
        var request = new ROSLIB.ServiceRequest({
            name : 'tunnel_height',
            value: ui.value
        });
        setImageProcessingParamClient.callService(request, function(result) {
        });
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

    var updateParamTunnelWidth = function(event,ui) {
        var request = new ROSLIB.ServiceRequest({
            name : 'tunnel_width',
            value: ui.value
        });
        setImageProcessingParamClient.callService(request, function(result) {
        });
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

    var updateParamTunnelYOffset = function(event,ui) {
        var request = new ROSLIB.ServiceRequest({
            name : 'tunnel_y_offset',
            value: ui.value
        });
        setImageProcessingParamClient.callService(request, function(result) {
        });
        $("#tunnelYOffsetDisplay").text(ui.value);
        var tunnelHeightSliderMax = imageHeight - ui.value;
        $("#tunnelHeightSlider").slider("option","max",tunnelHeightSliderMax);
        if ($('#tunnelHeightSlider').slider("option","value") > tunnelHeightSliderMax) {
            $('#tunnelHeightSlider').slider("option","value",tunnelHeightSliderMax);
            request = new ROSLIB.ServiceRequest({
                name : 'tunnel_height',
                value: tunnelHeightSliderMax
            });
            setImageProcessingParamClient.callService(request, function(result) {
            });
            $("#tunnelHeightDisplay").text(tunnelHeightSliderMax);
        };
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
        var request = new ROSLIB.ServiceRequest({
            name : 'gate_y_offsets',
            array: gateYOffsets
        });
        setImageProcessingArrayParamClient.callService(request, function(result) {
        });
        // paramGateYOffsets.set(gateYOffsets);
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
        var request = new ROSLIB.ServiceRequest({
            name : 'tunnel_x_offsets',
            array: tunnelXOffsets
        });
        setImageProcessingArrayParamClient.callService(request, function(result) {
        });
        // paramTunnelXOffsets.set(tunnelXOffsets);
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

    var saveImageProcessing = function() {
        var saveParamsRequest = new ROSLIB.ServiceRequest({
            params : "faa_image_processing"
        });
        saveParamsClient.callService(saveParamsRequest, function(result) {
        });
    };

    var updateImageSizeDisplay = function(event,ui) {
        var imageWidthNew = Math.round(imageWidth*(ui.value/100));
        var imageHeightNew = Math.round(imageHeight*(ui.value/100));
        $("#mjpegSettings").html('<img src="http://localhost:8080/stream?topic=/camera/image_processed?width='+imageWidthNew+'?height='+imageHeightNew+'" />');
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

    var refreshSaveBackgroundImages = function() {
        paramActuationInitialized.get(function(value) {
            if (value) {
                clearInterval(timerRefresh);
                $('#btnSaveBackgroundImages').button('option', 'label', 'Save Background Images');
                $('#btnSaveBackgroundImages').button('enable');
            };
        });
    };
    var refreshImageProcessing = function() {
        getImageProcessingParams();
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
            };
        };
    };
    var refreshExperimentParameters = function() {
        // paramExperimentName.get(updateExperimentName);
        paramAcclimationDuration.get(updateAcclimationDuration);
        paramStartChamberDuration.get(updateStartChamberDuration);
        paramTrialCount.get(updateTrialCount);
        paramAirBeforeDuration.get(updateAirBeforeDuration);
        paramEthanolDuration.get(updateEthanolDuration);
        paramEthanolVial.get(updateEthanolVial);
        paramAirAfterDuration.get(updateAirAfterDuration);
        paramLightsEnabled.get(updateLightsEnabled);
        paramLightsPercentCapacity.get(updateLightsPercentCapacity);
        paramLightsDurationOn.get(updateLightsDurationOn);
        paramLightsDurationOff.get(updateLightsDurationOff);
        paramTunnelsEnabled.get(updateTunnelsEnabled);
        paramTimeoutDuration.get(updateTimeoutDuration);
    };
    // refreshImageProcessing();

    // tab settings
    $('.progressbar').progressbar({
        value: 0
    });

    $(':button').button();

    // hardware info tab
    $('#btnInfo').bind('click', function() {
    });

    // save background images tab
    $('#btnSaveBackgroundImages').bind('click', function() {
        saveBackgroundImages.publish(emptyMsg);
        $('#btnSaveBackgroundImages').button('disable');
        tabs.tabs('enable',tabImageProcessingSettings);
        $('#saveBackgroundImagesInstructions').text('Click on Image Processing Settings tab to proceed...');
    });

    // flow rate settings tab
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
                               });

    // experiment controls tab
    var saveExperimentParameters = function() {
        var saveParamsRequest = new ROSLIB.ServiceRequest({
            params : "faa_experiment"
        });
        saveParamsClient.callService(saveParamsRequest, function(result) {
        });
    };
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
        acclimationDuration = param;
    };
    var updateParamAcclimationDuration = function(event, ui) {
        $('#acclimationDisplay').text(time_display_string(ui.value));
        paramAcclimationDuration.set(ui.value);
        acclimationDuration = ui.value;
    };
    $('#acclimationSlider').slider({
        range: "min",
        value: 120,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAcclimationDuration,
        stop: updateParamAcclimationDuration
    });
    var updateTrialCount = function(param) {
        $("#trialCountSlider").slider("option","value",param);
        $("#trialCountDisplay").text(param);
        trialCount = param;
    };
    var updateParamTrialCount = function(event, ui) {
        $('#trialCountDisplay').text(ui.value);
        paramTrialCount.set(ui.value);
        trialCount = ui.value;
    };
    $('#trialCountSlider').slider({
        range: "min",
        value: trialCount,
        min: 1,
        max: 10,
        step: 1,
        slide: updateParamTrialCount,
        stop: updateParamTrialCount
    });

    var updateStartChamberDuration = function(param) {
        $("#startChamberDurationSlider").slider("option","value",param);
        $("#startChamberDurationDisplay").text(time_display_string(param));
        startChamberDuration = param;
    };
    var updateParamStartChamberDuration = function(event, ui) {
        $('#startChamberDurationDisplay').text(time_display_string(ui.value));
        paramStartChamberDuration.set(ui.value);
        startChamberDuration = ui.value;
    };
    $('#startChamberDurationSlider').slider({
        range: "min",
        value: startChamberDuration,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamStartChamberDuration,
        stop: updateParamStartChamberDuration
    });

    // ethanol chamber settings
    $("#ethanolVialSelection").buttonset();
    var updateAirBeforeDuration = function(param) {
        $("#endChamberSliderAirBefore").slider("option","value",param);
        $("#endChamberDisplayAirBefore").text(time_display_string(param));
        waitTimeEndChambersAirBefore = param;
        update_wait_time_end_chambers();
    };
    var updateParamAirBeforeDuration = function(event, ui) {
        $('#endChamberDisplayAirBefore').text(time_display_string(ui.value));
        waitTimeEndChambersAirBefore = ui.value;
        update_wait_time_end_chambers();
        paramAirBeforeDuration.set(ui.value);
    };
    $('#endChamberSliderAirBefore').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAirBeforeDuration,
        stop: updateParamAirBeforeDuration
    });
    var updateEthanolDuration = function(param) {
        $("#endChamberSliderEthanol").slider("option","value",param);
        $("#endChamberDisplayEthanol").text(time_display_string(param));
        waitTimeEndChambersEthanol = param;
        update_wait_time_end_chambers();
    };
    var updateParamEthanolDuration = function(event, ui) {
        $('#endChamberDisplayEthanol').text(time_display_string(ui.value));
        waitTimeEndChambersEthanol = ui.value;
        update_wait_time_end_chambers();
        paramEthanolDuration.set(ui.value);
    };
    $('#endChamberSliderEthanol').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamEthanolDuration,
        stop: updateParamEthanolDuration
    });
    var updateAirAfterDuration = function(param) {
        $("#endChamberSliderAirAfter").slider("option","value",param);
        $("#endChamberDisplayAirAfter").text(time_display_string(param));
        waitTimeEndChambersAirAfter = param;
        update_wait_time_end_chambers();
    };
    var updateParamAirAfterDuration = function(event, ui) {
        $('#endChamberDisplayAirAfter').text(time_display_string(ui.value));
        waitTimeEndChambersAirAfter = ui.value;
        update_wait_time_end_chambers();
        paramAirAfterDuration.set(ui.value);
    };
    $('#endChamberSliderAirAfter').slider({
        range: "min",
        value: 0,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamAirAfterDuration,
        stop: updateParamAirAfterDuration
    });
    var update_wait_time_end_chambers = function() {
        waitTimeEndChambersTotal = waitTimeEndChambersAirBefore + 
            waitTimeEndChambersEthanol + 
            waitTimeEndChambersAirAfter;
        $('#endChamberDisplayTotalTime').text(time_display_string(waitTimeEndChambersTotal));
    };
    var updateLightsEnabled = function(param) {
        if (param) {
            $('#endChamberLightsPercentCapacity').show();
            $('#endChamberLightsDurationOn').show();
            $('#endChamberLightsDurationOff').show();
            $('#btnEndChamberLightsEnable').button('disable');
            $('#btnEndChamberLightsDisable').button('enable');
        } else {
            $('#endChamberLightsPercentCapacity').hide();
            $('#endChamberLightsDurationOn').hide();
            $('#endChamberLightsDurationOff').hide();
            $('#btnEndChamberLightsEnable').button('enable');
            $('#btnEndChamberLightsDisable').button('disable');
        };
    };
    $('#btnEndChamberLightsEnable').bind('click', function() {
        paramLightsEnabled.set(true);
        updateLightsEnabled(true);
    });
    $('#btnEndChamberLightsDisable').bind('click', function() {
        paramLightsEnabled.set(false);
        updateLightsEnabled(false);
    });
    var updateLightsPercentCapacity = function(param) {
        $("#endChamberSliderLightsPercentCapacity").slider("option","value",param);
        $("#endChamberDisplayLightsPercentCapacity").text(param + "%");
    };
    var updateParamLightsPercentCapacity = function(event, ui) {
        $('#endChamberDisplayLightsPercentCapacity').text(ui.value + "%");
        paramLightsPercentCapacity.set(ui.value);
    };
    $('#endChamberSliderLightsPercentCapacity').slider({
        range: "min",
        value: 0,
        min: 1,
        max: 100,
        step: 1,
        slide: updateParamLightsPercentCapacity,
        stop: updateParamLightsPercentCapacity
    });
    var updateLightsDurationOn = function(param) {
        $("#endChamberSliderLightsDurationOn").slider("option","value",param);
        $("#endChamberDisplayLightsDurationOn").text(param + "ms");
    };
    var updateParamLightsDurationOn = function(event, ui) {
        $('#endChamberDisplayLightsDurationOn').text(ui.value + "ms");
        paramLightsDurationOn.set(ui.value);
    };
    $('#endChamberSliderLightsDurationOn').slider({
        range: "min",
        value: 0,
        min: 1,
        max: 100,
        step: 1,
        // min: 100,
        // max: 10000,
        // step: 100,
        slide: updateParamLightsDurationOn,
        stop: updateParamLightsDurationOn
    });
    var updateLightsDurationOff = function(param) {
        $("#endChamberSliderLightsDurationOff").slider("option","value",param);
        $("#endChamberDisplayLightsDurationOff").text(param + "ms");
    };
    var updateParamLightsDurationOff = function(event, ui) {
        $('#endChamberDisplayLightsDurationOff').text(ui.value + "ms");
        paramLightsDurationOff.set(ui.value);
    };
    $('#endChamberSliderLightsDurationOff').slider({
        range: "min",
        value: 0,
        min: 1,
        max: 100,
        step: 1,
        // min: 100,
        // max: 10000,
        // step: 100,
        slide: updateParamLightsDurationOff,
        stop: updateParamLightsDurationOff
    });

    var updateEthanolVial = function(param) {
        // console.log("#ethanolVial"+param);
        $("#ethanolVial"+param).prop("checked", true ).button('refresh');
    };
    $("input:radio[name=vialEthanol]").click(function(){
        vialEthanol = parseInt($(this).attr("value"));
        paramEthanolVial.set(vialEthanol);
    });

    var update_mfc_flow_rate_settings = function(device,mfc,percent_capacity) {
        if ((device === 0) || (device === 1)) {
            $("#odor" + device + "Mfc" + mfc + "Slider").slider( "value", percent_capacity );
            $("#odor" + device + "Mfc" + mfc + "Display").text(percent_capacity);
        } else if (device === 2) {
            $("#ethanolMfc" + mfc + "Slider").slider( "value", percent_capacity );
            $("#ethanolMfc" + mfc + "Display").text(percent_capacity);
        }
    };

    var updateTimeoutDuration = function(param) {
        $("#timeoutSlider").slider("option","value",param);
        $("#timeoutDisplay").text(time_display_string(param));
        timeoutDuration = param;
    };
    var updateParamTimeoutDuration = function(event, ui) {
        $('#timeoutDisplay').text(time_display_string(ui.value));
        paramTimeoutDuration.set(ui.value);
        timeoutDuration = ui.value;
    };
    $('#timeoutSlider').slider({
        range: "min",
        value: 120,
        min: 0,
        max: 3600,
        step: 10,
        slide: updateParamTimeoutDuration,
        stop: updateParamTimeoutDuration
    });
    var updateTunnelsEnabled = function(param) {
        tunnelsEnabled = param;
        for (var tunnel=0; tunnel<tunnelsEnabled.length; tunnel++){
            if (tunnelsEnabled[tunnel]) {
                $('#btnDisable'+tunnel).button('enable');
                $('#btnDisable'+tunnel).fadeTo('slow',1);
                $('#btnReenable'+tunnel).button('disable');
                $('#btnReenable'+tunnel).fadeTo('slow',0);
            } else {
                $('#btnDisable'+tunnel).button('disable');
                $('#btnDisable'+tunnel).fadeTo('slow',0);
                $('#btnReenable'+tunnel).button('enable');
                $('#btnReenable'+tunnel).fadeTo('slow',1);
            }
        }
    };
    var updateExperimentTunnelsEnabled = function() {
        var msg = new ROSLIB.Message({
            tunnels_enabled: tunnelsEnabled
        });
        experimentTunnelsEnabled.publish(msg);
    };

    var initialize_tabs = function () {
        update_wait_time_end_chambers();
        $('#endChamberDisplayAirBefore').text(time_display_string(waitTimeEndChambersAirBefore));
        $('#endChamberSliderAirBefore').slider('value',waitTimeEndChambersAirBefore);
        $('#endChamberDisplayEthanol').text(time_display_string(waitTimeEndChambersEthanol));
        $('#endChamberSliderEthanol').slider('value',waitTimeEndChambersEthanol);
        $('#endChamberDisplayAirAfter').text(time_display_string(waitTimeEndChambersAirAfter));
        $('#endChamberSliderAirAfter').slider('value',waitTimeEndChambersAirAfter);
        $('#endChamberDisplayTotalTime').text(time_display_string(waitTimeEndChambersTotal));
        $('#acclimationDisplay').text(time_display_string(acclimationDuration));
        $('#timeoutDisplay').text(time_display_string(timeoutDuration));
        $('#trialCountDisplay').text(trialCount);
    };
    initialize_tabs();

    // General functions

    // Initialize button and timer states
    var initialize = function () {
        // $('#btnAbort').button('enable');
        // $('#btnAbort').fadeTo(0,1);
        $('#btnRunExperiment').button('disable');
        $('#btnRunExperiment').fadeTo(0,0);
        $('#btnLoadFlies').button('enable');
        $('#btnLoadFlies').fadeTo(0,1);
        $('.disable').button('enable');
        $('.disable').fadeTo(0,1);
        for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#btnDisable'+tunnel).text('Disable Tunnel ' + (tunnel + 1));
        }
        $('.reenable').button('disable');
        $('.reenable').fadeTo(0,0);
        for (tunnel=0;tunnel<tunnelCount;tunnel++) {
            $('#btnReenable'+tunnel).text('Reenable Tunnel ' + (tunnel + 1));
        }
        $("#tabs").tabs("option","disabled",[tabImageProcessingSettings,tabFlowRateSettings,tabExperimentSettings,tabExperimentControls]);
        $("#tabs").tabs("enable",tabSaveBackgroundImages);
        $("#tabs").tabs("option","active",tabSaveBackgroundImages);
        $('#btnSaveBackgroundImages').button('disable');
        $('#btnSaveBackgroundImages').button('option', 'label', 'Waiting for hardware to initialize...');
        $('#saveBackgroundImagesInstructions').text('Please check lighting conditions and make sure curtain is closed and that no flies are in the tunnels before saving background images.');
        $('#endChamberLightsPercentCapacity').hide();
        $('#endChamberLightsDurationOn').hide();
        $('#endChamberLightsDurationOff').hide();
        $('#btnEndChamberLightsEnable').button('enable');
        $('#btnEndChamberLightsDisable').button('disable');
    };
    initialize();

    // Experiment Step A: Load Flies
    $('#btnLoadFlies').bind('click', function() {
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        loadFlies.publish(emptyMsg);
        $('#btnRunExperiment').button('enable');
        $('#btnRunExperiment').fadeTo('slow',1);
    });

    // Experiment Step B: Run Experiment
    $('#btnRunExperiment').bind('click', function() {
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        runExperiment.publish(emptyMsg);
    });

    $('.disable').bind('click', function() {
        var tunnel = $(this).prop('value');
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        $('#btnReenable'+tunnel).button('enable');
        $('#btnReenable'+tunnel).fadeTo('slow',1);
        tunnelsEnabled[tunnel] = false;
        updateExperimentTunnelsEnabled();        
    });
    $('.reenable').bind('click', function() {
        var tunnel = $(this).prop('value');
        $(this).button('disable');
        $(this).fadeTo('slow',0);
        $('#btnDisable'+tunnel).button('enable');
        $('#btnDisable'+tunnel).fadeTo('slow',1);
        tunnelsEnabled[tunnel] = true;        
        updateExperimentTunnelsEnabled();        
    });

    // Abort
    // $('#btnAbort').bind('click', function() {
    //     $('#btnRunExperiment').button('disable');
    //     $('#btnRunExperiment').fadeTo('slow',0);
    //     $('#btnLoadFlies').button('enable');
    //     $('#btnLoadFlies').fadeTo('slow',1);
    // });

});

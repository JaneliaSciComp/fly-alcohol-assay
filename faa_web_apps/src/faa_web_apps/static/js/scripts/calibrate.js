$(function() {
    $(':button').button();

    // Initialize variables
    var tunnelCount = 6;
    var gateCount = 3;
    var tableCols = tunnelCount;
    $("td").css("width",100/tableCols + '%');
    var millisecondsInSecond = 1000;
    var timerRefresh;
    var periodRefresh = 500;
    var imageWidth = 1280;
    var imageHeight = 960;

    var tunnelSliderInitialized = [];
    var gateSliderInitialized = [];

    for (var tunnel=0;tunnel<tunnelCount;tunnel++) {
        tunnelSliderInitialized[tunnel] = false;
    };
    for (var gate=0;gate<gateCount;gate++) {
        gateSliderInitialized[gate] = false;
    };

    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    // var paramImageWidth = new ROSLIB.Param({
    //   ros : ros,
    //   name : '/image_processing/image_width'
    // });
    // var paramImageHeight = new ROSLIB.Param({
    //   ros : ros,
    //   name : '/image_processing/image_height'
    // });
    var paramTunnelWidth = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/tunnel_width'
    });
    var paramTunnelHeight = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/tunnel_height'
    });
    var paramTunnelXOffsets = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/tunnel_x_offsets'
    });
    var paramTunnelYOffset = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/tunnel_y_offset'
    });
    var paramGateHeight = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/gate_height'
    });
    var paramGateYOffsets = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/gate_y_offsets'
    });
    var paramCalibrationLineThickness = new ROSLIB.Param({
      ros : ros,
      name : '/image_processing/calibration_line_thickness'
    });

    var saveParamsClient = new ROSLIB.Service({
        ros : ros,
        name : '/save_params',
        serviceType : 'faa_utilities/SaveParams'
    });

    var saveImageProcessingParamsRequest = new ROSLIB.ServiceRequest({
        params : "image_processing"
    });

    var updateParamLineThickness = function(event, ui) {
        paramCalibrationLineThickness.set(ui.value);
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
        console.log(tunnelXOffsets);
        paramTunnelXOffsets.set(tunnelXOffsets);
    };
    $(".slider.calibration.xOffset").slider({
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
        max: 1000,
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
        console.log(gateYOffsets);
        paramGateYOffsets.set(gateYOffsets);
    };
    $(".slider.calibration.yOffset").slider({
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

    $('#btnSave').bind('click', function() {
        saveParamsClient.callService(saveImageProcessingParamsRequest, function(result) {
            alert('Result for service call on '
                  + saveParamsClient.name
                  + ': '
                  + result.status);
        });
    });

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

    var refresh = function() {
        // updateImageSizeDisplay();
        paramCalibrationLineThickness.get(updateLineThickness);
        paramTunnelWidth.get(updateTunnelWidth);
        paramTunnelXOffsets.get(updateTunnelXOffsets);
        paramTunnelHeight.get(updateTunnelHeight);
        paramTunnelYOffset.get(updateTunnelYOffset);
        paramGateHeight.get(updateGateHeight);
        paramGateYOffsets.get(updateGateYOffsets);
    };
    refresh();
    $('#btnRefresh').bind('click', function() {
        refresh();
    });

    var refreshCount = 0;
    var timer_refresh = function() {
        if (0<refreshCount) {
            refresh();
        }
        if (8<refreshCount) {
            clearInterval(timerRefresh);
        };
        refreshCount++;
        console.log(refreshCount);
    };
    // timerRefresh = setInterval(timer_refresh,periodRefresh);

});

function test_regiongrow(I)
compile_regiongrow;
% TODO possibly augment with non-local features, like adding pixels
% that are close to the mean pixel value of the region already found
% (that will basically generalize the idea of "diffuse" edges as embodied
% in SUMLO
hi = 0.0660;
lo =  0.0080; 
sumlo =  0.0420;
figure,
display()


uicontrol('Style','text',...
        'Position',[10 50 120 20],...
        'String','hi')
uicontrol('Style', 'slider', 'Callback', @hiCallback, 'Min', 0, 'Max', 0.2, 'SliderStep', [0.01 0.01], 'Value', 0.02, 'Position', [10 20 120 20]);

uicontrol('Style','text',...
        'Position',[140 50 120 20],...
        'String','lo')
uicontrol('Style', 'slider', 'Callback', @loCallback, 'Min', 0, 'Max', 0.1, 'SliderStep', [0.01 0.01], 'Value', 0.01, 'Position', [140 20 120 20]);

uicontrol('Style','text',...
        'Position',[270 50 120 20],...
        'String','sumlo')
uicontrol('Style', 'slider', 'Callback', @sumloCallback, 'Min', 0, 'Max', 0.2, 'SliderStep', [0.01 0.01], 'Value', 0.04, 'Position', [270 20 120 20]);


    function display()
        fimage = regiongrow(I, 'hi', hi, 'lo', lo, 'sumlo', sumlo);
        output = I;
        B = bwboundaries(fimage);
        
        fprintf('Redisplaying with hi: %.4f, lo: %.4f, sumlo: %.4f\n', hi, lo, sumlo);
        subplot(1,1,1), imshow(output), hold on, 
        for i = 1:length(B)
            boundary = B{i};
            row = boundary(:,1);
            col = boundary(:,2);
            plot(col,row, 'Color', 'yellow');
        end
        
    end

    function hiCallback(hObject, evt)
        %fprintf('Slider value is: %.4f\n', get(hObject, 'Value') );
        hi = get(hObject, 'Value');
        display()
    end
    function loCallback(hObject, evt)
        %fprintf('Slider value is: %.4f\n', get(hObject, 'Value') );
        lo = get(hObject, 'Value');
        display()
    end
    function sumloCallback(hObject, evt)
        %fprintf('Slider value is: %.4f\n', get(hObject, 'Value') );
        sumlo = get(hObject, 'Value');
        display()
    end
end



    

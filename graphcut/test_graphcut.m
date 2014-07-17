function test_graphcut(I)
compile_graphcut;
figure, 
display();


    function display()
        fimage = graphcut(I);
        output = I;
        B = bwboundaries(fimage);

        subplot(1,1,1), imshow(output), hold on, 
        for i = 1:length(B)
            boundary = B{i};
            row = boundary(:,1);
            col = boundary(:,2);
            plot(col,row, 'Color', 'yellow');
        end
        
    end
end



    

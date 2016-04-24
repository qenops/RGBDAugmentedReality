function poses = parseTracks(track)


 
        for i=1:size(track,2)
            
            avgTrack(i,:)=mean(track{1,i}(:,2:end),1);
            
                   poses(:,:,i)=reshape(avgTrack(i,:),[4 4 ])';
            
        end 
            
        



end
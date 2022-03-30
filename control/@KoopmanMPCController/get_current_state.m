function get_current_state(hController)
% This function retreives the last state estimate performed by the Kalman
% filter which is running in parallel
%
try%%
	% Read last estimate from file written by the estimator
	%
	strFilePath = hController.kalman.pathEstimate;
	%
	iNx = size(hController.model.ssM.model.A, 1);
	%
	fid = fopen([strFilePath, filesep, 'output.txt'],'r');
	fgetl(fid);
	afX0 = zeros(iNx,1);
	for i = 1:iNx
		afX0(i) = str2double(fgetl(fid));
	end
	hController.afX0 = afX0;
	fclose(fid);
	%
catch
	warning('Problem with the extraction of last state estimate')
end

end


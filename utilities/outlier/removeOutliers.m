function [ afDataFree ] = removeOutliers( afData )
	%
	[afData_NoOut, iOutIndex, ~] = deleteoutliers(afData, 0.05, 1);
	%
	iNotOutIndex = find(~isnan(afData_NoOut));
	
	for i = 1:length(iOutIndex)
		%
		iPreviousInde	= find(iOutIndex(i) > iNotOutIndex);
		iFollowInde		= find(iOutIndex(i) < iNotOutIndex);
		
		%
		if isempty(iPreviousInde)
			iPrevious = iNotOutIndex(1);
		else
			iPrevious		= iNotOutIndex(iPreviousInde(end));
		end
		%
		if isempty(iFollowInde)
			iFollowing = iNotOutIndex(end);
		else
			iFollowing		= iNotOutIndex(iFollowInde(1));
		end
		afData_NoOut(iOutIndex(i)) = (afData(iFollowing) + afData(iPrevious))/2;
		%
	end%%
	afDataFree = afData_NoOut;
end%%


function iIndices  = getIndicesForMean(iIter, iN)
	% Returns the indices to complete the prediction of the AGC signal.
	% BAsed on the assumption that AGC prediction is always 2 hours long
	iIndex = iIter+8;
	iIndices = iIndex:(iN+iIndex-1);
	iIndices = mod(iIndices-1, 96)+1;
end%%


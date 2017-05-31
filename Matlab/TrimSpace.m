function aedat = TrimSpace(aedat, newX, newY)

%{
This function take a structure which has been imported and trims the events
(polarity only for now ...) down to the params newX and newY. parameters
give the array size and one-based, but the address space is zero based.
 nexW and newY are tuples, containing the desired range. Addresses are then
 shifted to start at 0,0
%}

% Zero-base the params
newX = newX - 1;
newY = newY - 1;

dbstop if error

if isfield(aedat, 'data') && isfield(aedat.data, 'polarity')
    keepXLogical = aedat.data.polarity.x <= newX(2) & aedat.data.polarity.x >= newX(1);
    keepYLogical = aedat.data.polarity.y < newY(2) & aedat.data.polarity.y >= newY(1);
    keepLogical = keepXLogical & keepYLogical;
    aedat.data.polarity.x = aedat.data.polarity.x(keepLogical) - newX(1);
    aedat.data.polarity.y = aedat.data.polarity.y(keepLogical) - newY(1);
    aedat.data.polarity.polarity = aedat.data.polarity.polarity(keepLogical);
    aedat.data.polarity.timeStamp = aedat.data.polarity.timeStamp(keepLogical);
    % Need to handle 'reset'
    aedat.data.polarity.numEvents = length(aedat.data.polarity.polarity);
end
    
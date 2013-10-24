%% Misura della lunghezza media dei lanci

% seleziono i file delle particelle che hanno superato la soglia che mi
% interessano
[FileInput,PathNameInput] = uigetfile('*.dat','Seleziona il file threshold','MultiSelect','on');

if iscell(FileInput)
    Nlanci = length(FileInput);
else
    Nlanci = 1;
end

% salvo in un vettore tutte le distanze raggiunte da ogni particella
dlanci = zeros(Nlanci,1);

for ilancio = 1:Nlanci
    dati = load(fullfile(PathNameInput,FileInput{ilancio}));
    dlanci(ilancio) = dati(2);
end

% calcolo la distanza media percorsa della serie
media = mean(dlanci);

fprintf('*** *** *** *** *** *** *** *** *** *** *** *** *** *** *** ***\n')
fprintf('Distanza media %2.6f m\n', media);
fprintf('*** *** *** *** *** *** *** *** *** *** *** *** *** *** *** ***\n')

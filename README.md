# ForceFeedback
## Starting Sequence Branch
In deze branch wordty de startup en callibratie ontwikkeld.
veel plezier
### In in uitgangssignalen
IN: 
- encA (digitaal signaal van encoder)
- encB (digitaal signaal van encoder)    

UIT:
- motorIn1 (digitaal signaal van motor: HIGH/LOW, geeft met IN2 de draairichting van de motor)
- motorIn2 (digitaal signaal van motor)
- motorEnA (digitaal signaal van motor: geeft de snelheid van de motor: 0= off)  

Variabelen:
- int rotCounter
- int maxRot

### Werkwijze

#### Algemeen
Het nulpunt van de rotatie van het stuur wordt het meest rechtse punt, als naar links wordt gedraaid gaat **rotCounter** omhoog tot maxRot is bereikt (uiterst links).

De motor draait naar rechts.  
Om de 10 ms wordt gecheckt of het stuur effectief draait. Zoja: blijf draaien en checken, zoniet: stop de motor en defineer dit punt als het nulpunt: **rotCounter = 0**.  
De motor draait naar links, ondertussen gaat **rotCounter** omhoog.  
Om de 10 ms wordt gecheckt of het stuur effectief draait. Zoja: blijf draaien en checken, zoniet: stop de motor en defineer dit punt als de maximale rotatie: **maxRot = rotCounter**.  
#### De 'rotatiecheck'
Dit moet nog geprogrammeerd worden.  
2 manieren:  
- ofwel a.d.h.v. stroommeting kijken of er tegenkracht wordt geleverd.
- ofwel kijken of de status van de encoder is verandert t.o.v. vorige check.




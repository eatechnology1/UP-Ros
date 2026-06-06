const fs = require('fs');
const path = require('path');

function walk(dir) {
    let results = [];
    const list = fs.readdirSync(dir);
    list.forEach(function(file) {
        file = path.join(dir, file);
        const stat = fs.statSync(file);
        if (stat && stat.isDirectory()) { 
            results = results.concat(walk(file));
        } else if (file.endsWith('.vue')) {
            results.push(file);
        }
    });
    return results;
}

const vueFiles = walk('./src/pages').concat(walk('./src/components'));
let backgrounds = new Set();

vueFiles.forEach(file => {
    let content = fs.readFileSync(file, 'utf8');
    const styleRegex = /<style[^>]*>([\s\S]*?)<\/style>/g;
    
    let match;
    while ((match = styleRegex.exec(content)) !== null) {
        const styleContent = match[1];
        const bgRegex = /background:\s*(rgba\([^)]+\)|#[0-9a-fA-F]{3,6})/g;
        let bgMatch;
        while ((bgMatch = bgRegex.exec(styleContent)) !== null) {
            backgrounds.add(bgMatch[1].replace(/\s+/g, ' '));
        }
    }
});

console.log(Array.from(backgrounds).sort().join('\n'));

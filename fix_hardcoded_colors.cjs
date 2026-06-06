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
let changedFiles = 0;

vueFiles.forEach(file => {
    let content = fs.readFileSync(file, 'utf8');
    let original = content;

    const styleRegex = /<style[^>]*>([\s\S]*?)<\/style>/g;

    content = content.replace(styleRegex, (match, styleContent) => {
        let newStyle = styleContent
            // Eradicate dark backgrounds
            .replace(/background:\s*#2d2d2d/gi, 'background: var(--bg-surface-solid)')
            .replace(/background:\s*#1e1e1e/gi, 'background: var(--bg-surface)')
            .replace(/border(-[a-z]+)?:\s*([^;]*)#444/gi, 'border$1: $2var(--border-medium)')
            
            // Adapt hardcoded pastel text colors to use variables that work in both modes
            .replace(/color:\s*#22c55e/gi, 'color: var(--text-code)') 
            .replace(/color:\s*#fbbf24/gi, 'color: var(--text-warning, #d97706)') 
            .replace(/color:\s*#fca5a5/gi, 'color: var(--text-danger, #dc2626)') 
            .replace(/color:\s*#60a5fa/gi, 'color: var(--text-info, #2563eb)') 
            .replace(/color:\s*#a855f7/gi, 'color: var(--text-primary)') 
            .replace(/color:\s*#86efac/gi, 'color: var(--text-code)') 
            .replace(/color:\s*#fde68a/gi, 'color: var(--text-warning, #d97706)'); 
            
        return match.replace(styleContent, newStyle);
    });

    if (content !== original) {
        fs.writeFileSync(file, content);
        changedFiles++;
        console.log("Updated", file);
    }
});

console.log(`Changed ${changedFiles} files.`);
